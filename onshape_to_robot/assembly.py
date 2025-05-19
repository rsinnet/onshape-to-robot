from __future__ import annotations

import json
import re
from typing import (
    Annotated,
    Dict,
    Generator,
    List,
    Optional,
    Tuple,
)

import numpy as np
from annotated_types import Len

from .config import Config
from .expression import ExpressionParser
from .message import (
    bright,
    error,
    info,
    success,
    warning,
)
from .onshape_api.client import Client
from .robot import Joint

INSTANCE_IGNORE = -1

InstanceId = Annotated[str, Len(17, 17)]
OccurrencePath = Tuple[InstanceId, ...]


class Frame:
    """
    Represents a frame attached
    """

    def __init__(self, body_id: int, name: str, T_world_frame: np.ndarray):
        self.body_id: int = body_id
        self.name: str = name
        self.T_world_frame: np.ndarray = T_world_frame


class DOF:
    """
    Represents a DOF
    """

    def __init__(
        self,
        body1_id: int,
        body2_id: int,
        name: str,
        joint_type: str,
        T_world_mate: np.ndarray,
        limits: Optional[Tuple[float, float]],
        axis: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ):
        self.body1_id: int = body1_id
        self.body2_id: int = body2_id
        self.name: str = name
        self.joint_type: str = joint_type
        self.T_world_mate: np.ndarray = T_world_mate
        self.limits: Optional[Tuple[float, float]] = limits
        self.axis: np.ndarray = axis

        limits_str = ""
        if limits is not None:
            limits_str = f"[{round(limits[0], 3)}: {round(limits[1], 3)}]"
        print(success(f"+ Found DOF: {name} ({joint_type}) {limits_str}"))

    def flip(self, flip_limits: bool = True):
        if flip_limits and self.limits is not None:
            self.limits = (-self.limits[1], -self.limits[0])

        # Flipping the joint around X axis
        flip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.T_world_mate[:3, :3] = self.T_world_mate[:3, :3] @ flip

    def other_body(self, body_id: int):
        if body_id == self.body1_id:
            return self.body2_id
        elif body_id == self.body2_id:
            return self.body1_id
        else:
            raise Exception(f"ERROR: body {body_id} is not part of this DOF")


def is_two_feature_mate(
    feature: dict,
) -> Optional[Tuple[dict, OccurrencePath, OccurrencePath]]:
    """
    Check if the feature is a mate with two occurrences.
    """
    if feature["featureType"] == "mate" and not feature["suppressed"]:
        data = feature["featureData"]
        if (
            "matedEntities" not in data
            or len(data["matedEntities"]) != 2
            or len(data["matedEntities"][0]["matedOccurrence"]) == 0
            or len(data["matedEntities"][1]["matedOccurrence"]) == 0
        ):
            return None
        occurrence_A = tuple(data["matedEntities"][0]["matedOccurrence"])
        occurrence_B = tuple(data["matedEntities"][1]["matedOccurrence"])
        return data, occurrence_A, occurrence_B


class Assembly:
    """
    Main entry point to process an assembly
    """

    def __init__(self, config: Config):
        self.config: Config = config

        self.client = None  # use from_config() instead of __init__()
        self.expression_parser = ExpressionParser()
        self.expression_parser.variables_lazy_loading = self.load_variables

        self.document_id: str = config.document_id
        self.workspace_id: str | None = config.workspace_id
        self.version_id: str | None = config.version_id

        # All (raw) data from assembly
        self.assembly_data: dict = {}
        self.current_body_id: int = 0
        # Map an occurrence to a body id
        self.instance_body: Dict[OccurrencePath, int] = {}
        # Frames object
        self.frames: List[Frame] = []
        # Loop closure constraints
        self.closures: list = []
        # Degrees of freedom
        self.dofs: List[DOF] = []
        # Features data
        self.features: dict = {}
        # self.matevalues

        self.all_features_by_assembly_key: Dict[tuple, dict] = {}
        self.all_matevalues_by_assembly_key: Dict[tuple, Optional[dict]] = {}

        # Configuration values
        self.configuration_parameters: dict = {}
        # Dictionnary mapping items to their children in the tree
        self.tree_children: dict = {}
        # Root nodes
        self.root_nodes: list = []
        # Overriden link names
        self.link_names: Dict[int, str] = {}
        # Relation indexed by target joints, values are [source joint, ratio]
        self.relations: dict = {}

        # Every instance in the root assembly and subassemblies
        # is an occurrence in the root assembly
        self.occurrences: Dict[OccurrencePath, dict] = {}

    @classmethod
    def from_config(cls, config: Config) -> Assembly:
        """Load an assembly from a Config."""
        assembly = cls(config)
        assembly.client = Client(logging=False, creds=assembly.config.config_file)
        assembly.load()
        return assembly

    def build_maps(self) -> None:
        """Build maps useful for processing the assembly."""
        self.build_occurrences_map()
        self.build_instances_map()
        self.load_configuration()

    def load(self) -> None:
        """Load the assembly."""
        self.ensure_workspace_or_version()
        self.find_assembly()
        self.check_configuration()
        self.retrieve_assembly()
        self.load_features()
        self.populate_missing_instance_fields()

        self.build_maps()

        with open("features.json", "w") as stream:
            json.dump(self.features, stream, indent=4)

        self.process_mates()
        self.build_trees()
        self.find_relations()
        print("")

    def ensure_workspace_or_version(self):
        """
        Ensure either a workspace id or a version id is set
        If none, try to retrieve the current workspace ID from API
        """
        if self.version_id:
            print(bright(f"* Using configuration version ID {self.version_id} ..."))
        elif self.workspace_id:
            print(bright(f"* Using configuration workspace ID {self.workspace_id} ..."))
        else:
            print(
                bright(
                    "* Not workspace ID specified, retrieving the current workspace ..."
                )
            )
            document = self.client.get_document(self.config.document_id)
            self.workspace_id = document["defaultWorkspace"]["id"]
            print(success(f"+ Using workspace id: {self.workspace_id}"))

    def find_assembly(self):
        """
        Find the wanted assembly from the document
        """
        if self.config.element_id:
            print(
                bright(f"* Using configuration element ID {self.config.element_id} ...")
            )
            self.element_id = self.config.element_id
            return

        print(
            bright(
                "\n* Retrieving elements in the document, searching for the assembly..."
            )
        )

        elements = self.client.list_elements(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            "v" if self.version_id else "w",
        )

        self.element_id = None
        assemblies: dict = {}
        for element in elements:
            if element["type"] == "Assembly":
                assemblies[element["name"]] = element["id"]

        if self.config.assembly_name:
            if self.config.assembly_name in assemblies:
                self.element_id = assemblies[self.config.assembly_name]
            else:
                raise Exception(
                    f"ERROR: Unable to find required assembly {self.config.assembly_name} in this document"
                )
        else:
            if len(assemblies) == 0:
                raise Exception("ERROR: No assembly found in this document\n")
            elif len(assemblies) == 1:
                self.element_id = list(assemblies.values())[0]
            else:
                raise Exception(
                    f"ERROR: Multiple assemblies found, please specify the assembly name\n"
                    + '       to export (use "assemblyName" in the configuration file)\n'
                    + f"       Available assemblies: {', '.join(assemblies.keys())}"
                )

        if self.element_id == None:
            raise Exception(f"ERROR: Unable to find assembly in this document")

    def check_configuration(self):
        """
        Retrieve configuration items for given assembly and parsing config configuration
        """

        if self.config.configuration != "default":
            # Retrieving available config parameters
            elements = self.client.elements_configuration(
                self.document_id,
                self.version_id if self.version_id else self.workspace_id,
                self.element_id,
                wmv=("v" if self.version_id else "w"),
            )

            parameters = {}
            for entry in elements["configurationParameters"]:
                type_name = entry["typeName"]
                message = entry["message"]

                if type_name.startswith("BTMConfigurationParameterEnum"):
                    options = [
                        option["message"]["optionName"] for option in message["options"]
                    ]
                    parameters[message["parameterName"]] = [
                        "enum",
                        message["parameterId"],
                        options,
                    ]
                elif type_name.startswith("BTMConfigurationParameterBoolean"):
                    parameters[message["parameterName"]] = ["bool"]
                elif type_name.startswith("BTMConfigurationParameterQuantity"):
                    parameters[message["parameterName"]] = ["quantity"]

            # Parsing configuration
            parts = self.config.configuration.split(";")
            processed_configuration = []
            for part in parts:
                kv = part.split("=")
                if len(kv) == 2:
                    key, value = kv
                    if key not in parameters:
                        raise Exception(
                            f'ERROR: Unknown configuration parameter "{key}" in the configuration'
                        )
                    if parameters[key][0] == "enum":
                        if value not in parameters[key][2]:
                            raise Exception(
                                f'ERROR: Unknown value "{value}" for configuration parameter "{key}"'
                            )
                        key = parameters[key][1]
                    processed_configuration.append(f"{key}={value.replace(' ', '+')}")

            # Re-writing the configuration
            self.config.configuration = ";".join(processed_configuration)

    def retrieve_assembly(self):
        """
        Retrieve all assembly data
        """
        print(bright(f"* Retrieving assembly with id {self.element_id}"))

        self.assembly_data: dict = self.client.get_assembly(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.element_id,
            wmv=("v" if self.version_id else "w"),
            configuration=self.config.configuration,
        )

        with open("assembly.json", "w") as stream:
            json.dump(self.assembly_data, stream, indent=4)

        self.microversion_id: str = self.assembly_data["rootAssembly"][
            "documentMicroversion"
        ]

    def build_occurrences_map(self):
        """
        Create a map to look up occurrence by full path.
        """
        for occurrence in self.assembly_data["rootAssembly"]["occurrences"]:
            self.occurrences[tuple(occurrence["path"])] = occurrence

    def populate_missing_instance_fields(self) -> None:
        """
        Populate missing instance fields in the assembly data.

        It seems that when parts are repeated, they are numbered with
        <1>, <2>, <3> and all but <1> have most of their data missing.

        This method goes through and fills in the missing data by looking
        up the <1> instance in the list of instances in that assembly.
        """
        def copy_new_keys(source: dict, destination: dict) -> None:
            for key, value in source.items():
                if key not in destination:
                    destination[key] = value
            return destination

        def fill_instances(instances):
            for instance_to in instances:
                match = re.search(r"(.*) <(\d+)>$", instance_to["name"])
                if not match:
                    continue
                name = match.group(1)
                number = int(match.group(2))
                # We're assuming the first one has all the data and
                # subsequent have the duplicate data omitted.
                # TODO(RWS): What happens if there is a name collision?

                for instance_from in instances:
                    if instance_from["name"].startswith(name):
                        match_inner = re.search(r"<(\d+)>$", instance_from["name"])
                        if not match_inner:
                            continue
                        copy_new_keys(instance_from, instance_to)
                        break

        fill_instances(self.assembly_data["rootAssembly"]["instances"])
        for subassembly in self.assembly_data["subAssemblies"]:
            fill_instances(subassembly["instances"])

        with open("assembly2.json", "w", encoding="utf-8") as stream:
            json.dump(self.assembly_data, stream, indent=4)

    def walk_instances(
        self,
        include_suppressed: bool = False,
        prefix: Optional[OccurrencePath] = None,
        instances: Optional[List[dict]] = None,
    ) -> Generator[Tuple[OccurrencePath, dict]]:
        """Walk through all instances in the assembly."""
        if prefix is None:
            prefix = tuple()
        if instances is None:
            instances = self.assembly_data["rootAssembly"]["instances"]
        for instance in instances:
            if not include_suppressed:
                if instance["suppressed"]:
                    continue
            path = prefix + (instance["id"],)
            yield path, instance
            if instance["type"] == "Assembly":
                try:
                    sub_assembly = self.find_subassembly(instance)
                except ValueError:
                    # Suppressed assemblies may not be included in the
                    # assembly data, so we just skip them.
                    if instance["suppressed"]:
                        continue
                    raise
                yield from self.walk_instances(
                    include_suppressed, path, sub_assembly["instances"]
                )

    def build_instances_map(self):
        """Update occurrences so they include instance data."""
        for path, instance in self.walk_instances(include_suppressed=True):
            try:
                self.get_occurrence(path)["instance"] = instance
            except KeyError:
                # Some instances are not occurrences so we just ignore them.
                # This seems to be thinks like bolts.
                # TODO(RWS): Figure out what's going and handle or document.
                pass

    def load_features(self):
        """
        Load features
        """

        self.features = self.client.get_features(
            self.document_id,
            self.microversion_id,
            self.element_id,
            wmv="m",
            configuration=self.config.configuration,
        )
        import json

        if not self.version_id:
            # TODO: This should support microversion in the future
            self.matevalues = self.client.matevalues(
                self.document_id,
                self.workspace_id,
                self.element_id,
                configuration=self.config.configuration,
            )
        else:
            self.matevalues = None

    def load_configuration(self):
        """
        Load configuration parameters
        """

        self.variable_values = None

        # Extracting configuration v ariables
        parts = self.assembly_data["rootAssembly"]["fullConfiguration"].split(";")
        for part in parts:
            key_value = part.split("=")
            if len(key_value) == 2:
                key, value = key_value
                value = value.replace("+", " ")
                self.configuration_parameters[key] = value
                try:
                    param_value = self.expression_parser.eval_expr(value)
                    self.expression_parser.variables[key] = param_value
                except ValueError:
                    pass

    def load_variables(self):
        """
        Load variables values (only if needed) in the expression parser
        """
        variables = self.client.get_variables(
            self.document_id,
            self.version_id if self.version_id else self.workspace_id,
            self.element_id,
            wmv="v" if self.version_id else "w",
            configuration=self.config.configuration,
        )
        for entry in variables:
            for variable in entry["variables"]:
                self.expression_parser.variables[variable["name"]] = (
                    self.expression_parser.eval_expr(variable["value"])
                )

    def get_occurrence(self, path: OccurrencePath) -> dict:
        """
        Retrieve occurrence from its path
        """
        return self.occurrences[path]

    def get_occurrence_transform(self, path: OccurrencePath) -> np.ndarray:
        """
        Retrieve occurrence transform from its path
        """
        T_world_part = np.array(self.get_occurrence(path)["transform"]).reshape(4, 4)

        return T_world_part

    def cs_to_transformation(self, cs: dict) -> np.ndarray:
        """
        Convert a coordinate system to a transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = np.stack(
            (
                np.array(cs["xAxis"]),
                np.array(cs["yAxis"]),
                np.array(cs["zAxis"]),
            )
        ).T
        T[:3, 3] = cs["origin"]

        return T

    def get_mate_transform(self, mated_entity: dict):
        return self.cs_to_transformation(mated_entity["matedCS"])

    def make_body(self, path: OccurrencePath) -> None:
        """
        Make the given occurrence a body.
        """

        self.instance_body[path] = self.current_body_id
        self.current_body_id += 1

    def merge_bodies(self, occurrence_A: str, occurrence_B: str):
        # Ensure occurrences are body
        if occurrence_A not in self.instance_body:
            self.make_body(occurrence_A)
        if occurrence_B not in self.instance_body:
            self.make_body(occurrence_B)

        # Merging bodies
        body1_id = self.instance_body[occurrence_A]
        body2_id = self.instance_body[occurrence_B]
        if body1_id > body2_id:
            body1_id, body2_id = body2_id, body1_id
        # print(
        #     f"Merging bodies ({body1_id} <> {body2_id}): "
        #     f"`{occurrence_A}` and `{occurrence_B}`"
        # )

        for occurrence in self.instance_body:
            if self.instance_body[occurrence] == body2_id:
                self.instance_body[occurrence] = body1_id

        for dof in self.dofs:
            if dof.body1_id == body2_id:
                dof.body1_id = body1_id
            if dof.body2_id == body2_id:
                dof.body2_id = body1_id

    def translation(self, x: float, y: float, z: float) -> np.ndarray:
        return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    def find_subassembly(self, key: Dict[str, str]) -> dict:
        """Look up a subassembly by its did/mid/eid."""
        did = key["documentId"]
        mid = key["documentMicroversion"]
        eid = key["elementId"]
        for subassembly in self.assembly_data["subAssemblies"]:
            if (
                did == subassembly["documentId"]
                and mid == subassembly["documentMicroversion"]
                and eid == subassembly["elementId"]
            ):
                return subassembly
        raise ValueError(f"Subassembly not found: d/{did}/m/{mid}/e/{eid}")

    def get_first_part_instance(self) -> dict:
        """Find the first part instance in the assembly.

        The part can then be used to create the root body.
        When designing in Onshape, it will be the first *part* in the
        instances list.

        Requires build_maps() to be called first.

        """
        for path, instance in self.walk_instances():
            instance_type = instance["type"]
            if instance_type == "Assembly":
                continue
            elif instance_type == "Part":
                return self.get_occurrence(path)

            # TODO(RWS): Handle other types of instances explicitly.
            raise RuntimeError(
                f"First non-assembly occurrence had type '{instance_type}'"
            )

    @staticmethod
    def process_joint_name_and_check_if_inverted(data: dict) -> str:
        """Extract joint name from mate name and set inverted property."""
        parts = data["name"].split("_")
        del parts[0]
        inverted = False
        if parts[-1] == "inv" or parts[-1] == "inverted":
            inverted = True
            del parts[-1]
        name = "_".join(parts)

        if name == "":
            raise RuntimeError(
                f"ERROR: the following dof should have a name {data['name']}"
            )
        return name, inverted

    def process_joint_type_and_limits(
        self, data: dict
    ) -> Tuple[str, Optional[Tuple[float, float]]]:
        """Extract joint type and limits from mate data."""
        limits = None
        if data["mateType"] == "REVOLUTE" or data["mateType"] == "CYLINDRICAL":
            if "wheel" in data["name"] or "continuous" in data["name"]:
                joint_type = Joint.CONTINUOUS
            else:
                joint_type = Joint.REVOLUTE

            if not self.config.ignore_limits:
                limits = self.get_limits(joint_type, data["name"])
        elif data["mateType"] == "SLIDER":
            joint_type = Joint.PRISMATIC
            if not self.config.ignore_limits:
                limits = self.get_limits(joint_type, data["name"])
        elif data["mateType"] == "FASTENED":
            joint_type = Joint.FIXED
        elif data["mateType"] == "BALL":
            joint_type = Joint.BALL
            if not self.config.ignore_limits:
                limits = self.get_limits(joint_type, data["name"])
        else:
            raise RuntimeError(
                f"ERROR: {data['name']} is declared as a DOF but the mate type is {data['mateType']}\n"
                + "       Only REVOLUTE, CYLINDRICAL, SLIDER and FASTENED are supported"
            )
        return joint_type, limits

    def merge_fixed_bodies(self) -> None:
        """
        Merge bodies that are rigidly connected to each other.
        """
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            if data["name"].startswith("fix_") or (
                data["mateType"] == "FASTENED"
                and not data["name"].startswith("dof_")
                and not data["name"].startswith("closing_")
                and not data["name"].startswith("frame_")
            ):
                self.merge_bodies(occurrence_A, occurrence_B)

        # Process mate groups.
        # for data, occurrences
        pass

    def process_frames(self) -> None:
        """Find all the frames.

        Locate each frame defined in the assembly and add to self.frames.
        Also merges the body of the frame part with its parent if
        draw_frames is enabled, otherwise sets the frame instance to be
        ignored.
        """
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            if data["name"].startswith("frame_"):
                name = "_".join(data["name"].split("_")[1:])
                if (
                    occurrence_A not in self.instance_body
                    and occurrence_B in self.instance_body
                ):
                    parent, child = occurrence_B, occurrence_A
                    mated_entity = data["matedEntities"][0]
                elif (
                    occurrence_B not in self.instance_body
                    and occurrence_A in self.instance_body
                ):
                    parent, child = occurrence_A, occurrence_B
                    mated_entity = data["matedEntities"][1]
                else:
                    raise RuntimeError(
                        f"Frame {name} should mate an orphan body to a body in the kinematics tree"
                    )

                T_world_part = self.get_occurrence_transform(child)

                self.frames += [Frame(self.instance_body[parent], name, T_world_part)]

                if self.config.draw_frames:
                    self.merge_bodies(parent, child)
                else:
                    self.instance_body[child] = INSTANCE_IGNORE

    def process_mates(self):
        """
        Pre-assign all non-assembly instances to a separate body id
        """
        # NOTE(RWS): Originally, this function treated each top-level
        # instance as a body, but now it treats each occurrence that
        # is not an assembly as a body.

        # top_level_instances = self.assembly_data["rootAssembly"]["instances"]
        # self.make_body(top_level_instances[0]["id"])

        # Find the first part occurrence, which may be in a subassembly
        # and create the first body, which will become the root body.
        first_part = self.get_first_part_instance()
        print(bright(f"* Found first part: {first_part}"))
        self.make_body(tuple(first_part["path"]))

        # We first search for DOFs
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            print(f"zzz occurrence_A: {occurrence_A}, occurrence_B: {occurrence_B}")

            if data["name"].startswith("dof_"):
                name, inverted = Assembly.process_joint_name_and_check_if_inverted(data)
                joint_type, limits = self.process_joint_type_and_limits(data)

                # We compute the axis in the world frame
                mated_entity = data["matedEntities"][0]
                T_world_part = self.get_occurrence_transform(occurrence_A)

                # jointToPart is the (rotation only) matrix from joint to the part
                # it is attached to
                T_part_mate = self.get_mate_transform(mated_entity)

                T_world_mate = T_world_part @ T_part_mate

                # Ensure occurrences are body
                if occurrence_A not in self.instance_body:
                    self.make_body(occurrence_A)
                if occurrence_B not in self.instance_body:
                    self.make_body(occurrence_B)

                dof = DOF(
                    self.instance_body[occurrence_A],
                    self.instance_body[occurrence_B],
                    name,
                    joint_type,
                    T_world_mate,
                    limits,
                )
                if inverted:
                    dof.flip()

                self.dofs.append(dof)

        self.merge_fixed_bodies()
        self.process_frames()

        # Checking that all instances are assigned to a body

        for path, occurrence in self.occurrences.items():
            # In some cases, instances only have name and id, so suppress them
            instance = occurrence["instance"]
            if "suppressed" not in instance:
                print(f"? Skipping an instance without suppressed: {instance}")
                continue
            if instance["suppressed"]:
                continue
            if instance["type"] == "Assembly":
                continue

            if path not in self.instance_body:
                print(
                    f"ERROR: Instance {path} of type {instance['type']} is not assigned to a body"
                )
                print(f"  instance: {instance}")
                print(f"  occurrence: {occurrence}")
                self.make_body(path)

        # Processing loop closing frames
        for data, occurrence_A, occurrence_B in self.feature_mating_two_occurrences():
            is_hinge_closure = data["mateType"] == "REVOLUTE"

            if data["name"].startswith("closing_"):
                for k in 0, 1:
                    mated_entity = data["matedEntities"][k]
                    # TODO(RWS): This probably won't work for subassemblies.
                    occurrence = mated_entity["matedOccurrence"][0]

                    T_world_part = self.get_occurrence_transform(
                        mated_entity["matedOccurrence"]
                    )
                    T_part_mate = self.get_mate_transform(mated_entity)
                    T_world_mate = T_world_part @ T_part_mate

                    self.frames.append(
                        Frame(
                            self.instance_body[occurrence],
                            f"{data['name']}_{k+1}",
                            T_world_mate,
                        )
                    )

                    if is_hinge_closure:
                        self.frames.append(
                            Frame(
                                self.instance_body[occurrence],
                                f"{data['name']}_{k+1}_z",
                                T_world_mate @ self.translation(0, 0, 0.1),
                            )
                        )

                closure_types = {
                    "FASTENED": "fixed",
                    "REVOLUTE": "revolute",
                    "BALL": "ball",
                    "SLIDER": "slider",
                }

                self.closures.append(
                    [
                        closure_types.get(data["mateType"], "unknown"),
                        f"{data['name']}_1",
                        f"{data['name']}_2",
                    ]
                )
                if is_hinge_closure:
                    self.closures.append(
                        [
                            closure_types.get(data["mateType"], "unknown"),
                            f"{data['name']}_1_z",
                            f"{data['name']}_2_z",
                        ]
                    )

        # Search for mate connector named "link_..." to override link names
        for feature in self.assembly_data["rootAssembly"]["features"]:
            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("link_"):
                link_name = "_".join(feature["featureData"]["name"].split("_")[1:])
                body_id = self.instance_body[
                    tuple(feature["featureData"]["occurrence"])
                ]
                self.link_names[body_id] = link_name

            if feature["featureType"] == "mateConnector" and feature["featureData"][
                "name"
            ].startswith("frame_"):
                name = "_".join(feature["featureData"]["name"].split("_")[1:])
                occurrence = tuple(feature["featureData"]["occurrence"])
                T_world_occurrence = self.get_occurrence_transform(occurrence)
                body_id = self.instance_body[occurrence]
                T_occurrence_mate = self.cs_to_transformation(
                    feature["featureData"]["mateConnectorCS"]
                )
                T_world_mate = T_world_occurrence @ T_occurrence_mate
                self.frames.append(Frame(body_id, name, T_world_mate))

        print(success(f"* Found total {len(self.dofs)} degrees of freedom"))

    def build_trees(self):
        """
        Perform checks on the produced tree
        """
        self.body_in_tree = []
        print(f"Inst bodies: {len(self.instance_body.values())}")
        print(f"ib vals: {self.instance_body.values()}")
        for body_id in self.instance_body.values():
            if body_id != INSTANCE_IGNORE and body_id not in self.body_in_tree:
                print(f"Build tree {body_id}")
                self.build_tree(body_id)
            # else:
            #     print(f"body_id: {body_id}")

        print(success(f"* Found {len(self.root_nodes)} root nodes {self.root_nodes}:"))
        for root_node in self.root_nodes:
            body_instance = self.body_instance(root_node)
            print(success(f"  - {body_instance['name']}"))

    def build_tree(self, root_node: int):
        """
        Building a tree starting at a root_node
        """
        print(f"Building tree for body_id: {root_node}")

        # Append the root node
        self.root_nodes.append(root_node)

        # Checking that the graph is actually a tree (no loop)
        exploring = [root_node]
        dofs = self.dofs.copy()
        while len(exploring) > 0:
            current = exploring.pop()
            print(f"Exploring {current}")
            self.body_in_tree.append(current)

            children = []
            dofs_to_remove = []
            for dof in dofs:
                if dof.body1_id == current:
                    dof.flip(flip_limits=False)
                    children.append(dof.body2_id)
                    dofs_to_remove.append(dof)
                elif dof.body2_id == current:
                    children.append(dof.body1_id)
                    dofs_to_remove.append(dof)
            for dof in dofs_to_remove:
                dofs.remove(dof)

            self.tree_children[current] = children
            for child in children:
                if child in self.body_in_tree:
                    raise Exception(
                        "The DOF graph is not a tree, check for loops in your DOFs"
                    )
                elif child not in exploring:
                    exploring.append(child)

    def walk_features(
        self, include_suppressed=False
    ) -> Generator[Tuple[OccurrencePath, dict]]:
        path = tuple()
        for feature in self.assembly_data["rootAssembly"]["features"]:
            if not include_suppressed and feature["suppressed"]:
                continue
            yield path, feature

        for path, occurrence in self.occurrences.items():
            if not include_suppressed and occurrence["instance"]["suppressed"]:
                continue
            if occurrence["instance"]["type"] == "Assembly":
                try:
                    subassembly = self.find_subassembly(occurrence["instance"])
                except ValueError:
                    # Suppressed assemblies may not be included in the
                    # assembly data, so we just skip them.
                    # TODO(RWS): Add a unit test.
                    if occurrence["instance"]["suppressed"]:
                        continue
                    raise
                for feature in subassembly["features"]:
                    yield path, feature

    def feature_mating_two_occurrences(
        self,
    ) -> Generator[Tuple[str, OccurrencePath, OccurrencePath]]:
        """
        Return all features that mate two occurrences at any assembly level.

        The returned occurrences are full paths, i.e., relative to the
        root assembly even if the feature was found in a subassembly.
        """

        for path, feature in self.walk_features():
            result = is_two_feature_mate(feature)
            if result is None:
                continue
            [data, occurrence_A, occurrence_B] = result
            yield data, path + occurrence_A, path + occurrence_B

    def get_feature_by_id(self, feature_id: str):
        """
        Find a specific feature by its ID
        """
        for feature in self.features["features"]:
            if feature["message"]["featureId"] == feature_id:
                return feature

        return None

    def find_relations(self):
        """
        Finding relations features in the assembly
        """
        for feature in self.features["features"]:
            if feature["typeName"] == "BTMMateRelation":
                relation_name = feature["message"]["name"]

                mated_dofs = None
                ratio = None
                reverse = None
                for parameter in feature["message"]["parameters"]:
                    if parameter["message"]["parameterId"] == "matesQuery":
                        queries = parameter["message"]["queries"]
                        if len(queries) == 2:
                            dof1 = self.get_feature_by_id(
                                queries[0]["message"]["featureId"]
                            )["message"]["name"]
                            dof2 = self.get_feature_by_id(
                                queries[1]["message"]["featureId"]
                            )["message"]["name"]
                            if dof1.startswith("dof_") and dof2.startswith("dof_"):
                                mated_dofs = [dof1[4:], dof2[4:]]
                    elif parameter["message"]["parameterId"] == "relationRatio":
                        ratio = self.read_expression(parameter["message"]["expression"])
                    elif parameter["message"]["parameterId"] == "reverseDirection":
                        reverse = parameter["message"]["value"]

                if mated_dofs is not None and ratio is not None and reverse is not None:
                    if not reverse:
                        ratio = -ratio

                    print(
                        success(
                            f"+ Found relation {relation_name} mating {mated_dofs} with ratio {ratio}"
                        )
                    )
                    if mated_dofs[1] in self.relations:
                        print(
                            warning(
                                f"Multiple relations found with {mated_dofs[1]} as target"
                            )
                        )

                    self.relations[mated_dofs[1]] = [mated_dofs[0], ratio]

    def read_parameter_value(self, parameter: str, name: str):
        """
        Try to read a parameter value from Onshape
        """

        # This is an expression
        if parameter["typeName"] == "BTMParameterNullableQuantity":
            return self.read_expression(parameter["message"]["expression"])
        if parameter["typeName"] == "BTMParameterConfigured":
            message = parameter["message"]
            parameterValue = self.configuration_parameters[
                message["configurationParameterId"]
            ]

            for value in message["values"]:
                if value["typeName"] == "BTMConfiguredValueByBoolean":
                    booleanValue = parameterValue == "true"
                    if value["message"]["booleanValue"] == booleanValue:
                        return self.read_expression(
                            value["message"]["value"]["message"]["expression"]
                        )
                elif value["typeName"] == "BTMConfiguredValueByEnum":
                    if value["message"]["enumValue"] == parameterValue:
                        return self.read_expression(
                            value["message"]["value"]["message"]["expression"]
                        )
                else:
                    raise Exception(
                        "Can't read value of parameter {name} configured with {value['typeName']}"
                    )

            print(error(f"Coud not find the value for {name}"))
        else:
            raise Exception(f"Unknown feature type for {name}: {parameter['typeName']}")

    def read_expression(self, expression: str):
        """
        Reading an expression from Onshape
        """
        return self.expression_parser.eval_expr(expression)

    def get_offset(self, name: str):
        """
        Retrieve the offset from current joint position in the assembly
        Currently, this only works with workspace in the API
        """
        if self.matevalues is None:
            return None

        for entry in self.matevalues["mateValues"]:
            if entry["mateName"] == name:
                if "rotationZ" in entry:
                    return entry["rotationZ"]
                elif "translationZ" in entry:
                    return entry["translationZ"]
                else:
                    print(warning(f"Unknown offset type for {name}"))
        return None

    def get_limits(self, joint_type: str, name: str) -> Tuple[float, float]:
        """
        Retrieve (low, high) limits for a given joint, if any
        """
        enabled = False
        minimum, maximum = 0, 0
        for feature in self.features["features"]:
            # Find coresponding joint
            if name == feature["message"]["name"]:
                # Find min and max values
                for parameter in feature["message"]["parameters"]:
                    if parameter["message"]["parameterId"] == "limitsEnabled":
                        enabled = parameter["message"]["value"]

                if enabled:
                    for parameter in feature["message"]["parameters"]:
                        if joint_type == Joint.REVOLUTE:
                            if parameter["message"]["parameterId"] == "limitAxialZMin":
                                minimum = self.read_parameter_value(parameter, name)
                            if parameter["message"]["parameterId"] == "limitAxialZMax":
                                maximum = self.read_parameter_value(parameter, name)
                        elif joint_type == Joint.PRISMATIC:
                            if parameter["message"]["parameterId"] == "limitZMin":
                                minimum = self.read_parameter_value(parameter, name)
                            if parameter["message"]["parameterId"] == "limitZMax":
                                maximum = self.read_parameter_value(parameter, name)
                        elif joint_type == Joint.BALL:
                            if (
                                parameter["message"]["parameterId"]
                                == "limitEulerConeAngleMax"
                            ):
                                minimum = 0
                                maximum = self.read_parameter_value(parameter, name)
                        else:
                            print(
                                warning(
                                    f"WARNING: Can't read limits for a joint of type {joint_type}"
                                )
                            )
                            print(parameter)
        if enabled:
            if joint_type != Joint.BALL:
                offset = self.get_offset(name)
                if offset is not None:
                    minimum -= offset
                    maximum -= offset
            return (minimum, maximum)
        else:
            if joint_type != Joint.CONTINUOUS:
                print(
                    warning(f"WARNING: joint {name} of type {joint_type} has no limits")
                )
            return None

    def body_instance(self, body_id: int):
        """
        Get the (first) instance associated with a given body
        """
        print(f"body_instance {body_id}")
        for path, instance in self.walk_instances():
            if instance["type"] == "Assembly":
                continue

            try:
                if self.instance_body[path] == body_id:
                    # print(f"Returning instance {path} from walk_instances.")
                    return instance
            except KeyError:
                # If it's not in the instance body map, it probably means
                # it's suppressed or not a body.
                raise

        print(f"Failed to find instance for body_id: {body_id}")
        # print(f"instance_body: {self.instance_body}")
        return None

    def body_occurrences(self, body_id: int):
        """
        Retrieve all occurrences associated to a given body id
        """
        for occurrence in self.assembly_data["rootAssembly"]["occurrences"]:
            key = tuple(occurrence["path"])
            if key in self.instance_body and self.instance_body[key] == body_id:
                yield occurrence

    def get_dof(self, body1_id: int, body2_id: int):
        """
        Get a DOF for given bodies
        """
        for dof in self.dofs:
            if (dof.body1_id == body1_id and dof.body2_id == body2_id) or (
                dof.body1_id == body2_id and dof.body2_id == body1_id
            ):
                return dof

        raise Exception(f"ERROR: no DOF found between {body1_id} and {body2_id}")
