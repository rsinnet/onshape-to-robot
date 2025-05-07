"""Test the end-to-end tool."""

import json
import logging
import os
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase

import yaml
from parameterized import parameterized  # type: ignore[import-untyped]

# INTEGRATION TEST TEMPLATE
#
# Each test runs the export on an Onshape Assembly with specified settings
# and represents an end-to-end test but with the intention of exercising
# specific functionality.
#
# The output is compared to an expected result by simply comparing the
# text output. This method is not robust to minor changes in how the API
# or tool works, but it is a quick way to get initial coverage without
# having to handle parsing and the math involved in more detailed comparison.


def _load_test_cases():
    path = Path(__file__).parent / "integration_test_cases.yaml"
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.load(stream, Loader=yaml.FullLoader)
    return [(x["name"], x["url"], x["expected"]) for x in data]


class TestIntegration(TestCase):

    def setUp(self) -> None:
        self.maxDiff = None
        self._output_dir = TemporaryDirectory()
        os.environ["ONSHAPE_TO_ROBOT_OUTPUT_DIR"] = str(self.output_dir)

    def tearDown(self) -> None:
        self._output_dir.cleanup()

    def _make_config(self, **kwargs) -> None:
        config = {
            "outputFormat": "urdf",
            "packageName": "mybot_description",
            "packageType": "ament",
            "robotName": "mybot",
            "addDummyBaseLink": True,
            "ignoreLimits": True,
        } | kwargs
        logging.info("Config: %s", config)
        with open(self.output_dir / "config.json", "w") as stream:
            json.dump(config, stream)
        self.config = config

    def _load_urdf(self) -> str:
        urdf_path = self.output_dir / "urdf" / "robot.urdf"
        with open(urdf_path, "r", encoding="utf-8") as stream:
            return stream.read()

    @property
    def output_dir(self) -> Path:
        """Get the current output directory."""
        return Path(self._output_dir.name)

    def _run_tool(self) -> None:
        command = ["python", "-m", "onshape_to_robot.onshape_to_robot", self.output_dir]
        process = subprocess.run(command, capture_output=False, text=True, check=True)

    @parameterized.expand(_load_test_cases())
    def test_box_assembly(self, name, url, expected):
        """Export a single box assembly."""
        self._make_config(documentsUrl=url)
        self._run_tool()

        # Print out the files in the output directory.
        output_files = os.listdir(self.output_dir)
        logging.info("Output files: %s", str(output_files))

        actual = self._load_urdf()

        logging.info(actual)
        logging.info(expected)

        self.assertEqual(expected, actual)
