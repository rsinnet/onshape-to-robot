"""Test the end-to-end tool."""

import logging
import os
import subprocess
from pathlib import Path

import yaml
from parameterized import parameterized  # type: ignore[import-untyped]
from onshape_to_robot.testing import TestHarness

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


class TestIntegration(TestHarness):

    def setUp(self) -> None:
        self.maxDiff = None
        super().setUp()

    def _run_tool(self) -> None:
        command = ["python", "-m", "onshape_to_robot.export", self.output_dir]
        process = subprocess.run(command, capture_output=False, text=True, check=True)

    @parameterized.expand(_load_test_cases())
    def test_integration(self, name, url, expected):
        """Export a single box assembly."""
        self.make_config(url=url)
        self._run_tool()

        # Print out the files in the output directory.
        output_files = os.listdir(self.output_dir)
        logging.info("Output files: %s", str(output_files))

        actual = self.load_urdf()

        logging.info("Expected:\n%s", expected)
        logging.info("Actual:\n%s", actual)

        self.copy_config_dir(overwrite=True)

        self.assertEqual(expected, actual)
