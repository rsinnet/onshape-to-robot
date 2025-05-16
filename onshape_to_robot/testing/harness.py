"""This test harness provides common test functionality."""

import json
import logging
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from typing import Optional
from ..config import Config


class TestHarness(TestCase):
    """Test harness for common test functionality."""

    def setUp(self):
        """Set up the test case."""
        self._output_dir = TemporaryDirectory()
        os.environ["ONSHAPE_TO_ROBOT_OUTPUT_DIR"] = str(self.output_dir)

    def tearDown(self):
        """Tear down the test case."""
        # Add any teardown code here
        pass

    @property
    def output_dir(self) -> Path:
        """Get the current output directory."""
        return Path(self._output_dir.name)

    def load_urdf(self) -> str:
        """Load the URDF file from the output directory."""
        urdf_path = self.output_dir / "urdf" / "robot.urdf"
        with open(urdf_path, "r", encoding="utf-8") as stream:
            return stream.read()


    def make_config(self, **kwargs) -> None:
        """Create a temp dir and configuration file for a test."""
        config = {
            "outputFormat": "urdf",
            "packageName": "mybot_description",
            "packageType": "ament",
            "robotName": "mybot",
            "addDummyBaseLink": True,
            "ignoreLimits": True,
            "drawFrames": False,
        } | kwargs
        logging.info("Config: %s", config)
        config_filepath = self.output_dir / "config.json"
        with open(config_filepath, "w", encoding="utf-8") as stream:
            json.dump(config, stream)
        return Config(str(self.output_dir))

    def delete_config_dir(self, path: Path) -> None:
        """Delete a config directory.

        This method safely deletes a config directory by only deleting
        specified files and directories, not the entire directory.
        """
        to_delete = [
            "CMakeLists.txt",
            "config.json",
            ".flake8",
            "launch",
            "meshes",
            "package.xml",
            "pyproject.toml",
            "rviz",
            "urdf",
        ]
        for item in to_delete:
            item_path = path / item
            if item_path.exists():
                if item_path.is_dir():
                    os.system(f"rm -rf {item_path}")
                else:
                    os.system(f"rm {item_path}")


    def copy_config_dir(
        self,
        dest: Optional[Path] = None,
        overwrite: bool = False,
    ) -> None:
        """Copy the config directory to the specified location.

        This method is great for examining test outputs, which would
        otherwise be deleted at the end of each test.
        """
        if dest is None:
            # Default to creating a folder in whatever directory is
            # set in the environment for the user's convenience.
            dest = Path.cwd() / "last_test_config"

        if not overwrite and dest.exists():
            raise FileExistsError(
                f"Destination {dest} already exists and overwrite is disabled."
            )

        dest.mkdir(parents=True, exist_ok=True)
        self.delete_config_dir(dest)

        # Copy all files and subdirectories from self.output_dir to dest
        for item in self.output_dir.iterdir():
            dest_item = dest / item.name
            if item.is_dir():
                # Recursively copy directories
                os.system(f"cp -r {item} {dest_item}")
            else:
                # Copy files
                os.system(f"cp {item} {dest_item}")
