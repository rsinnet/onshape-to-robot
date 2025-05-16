"""Test the Assembly class."""

import logging
from pathlib import Path

import commentjson as json
from parameterized import parameterized  # type: ignore[import-untyped]

from onshape_to_robot.assembly import Assembly
from onshape_to_robot.config import Config
from onshape_to_robot.testing import TestHarness


def _load_test_cases():
    path = Path(__file__).parent / "feature_mating_test_cases.json"
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    return [
        (
            x["name"],
            x["input"]["assembly"],
            x["input"]["features"],
            x["expected"],
        )
        for x in data
    ]


class TestAssembly(TestHarness):

    @parameterized.expand(_load_test_cases())
    def test_feature_mating_two_occurrences(
        self, name, assembly_data, features, expected
    ):

        config = self.make_config(
            document_id="43aaee52dd90f8f1b794a9ae",
            element_id="885eeed9f23c6ef38371a3d0",
            document_microversion="a7f0ea20048ff27ddad7ad40",
        )
        assembly = Assembly(config)
        assembly.assembly_data = assembly_data
        assembly.features = features
        mates = [
            {"data": data, "occurrenceA": occurrenceA, "occurrenceB": occurrenceB}
            for data, occurrenceA, occurrenceB in assembly.feature_mating_two_occurrences()
        ]
        logging.info("Expected: %s", json.dumps(expected))
        logging.info("Actual: %s", json.dumps(mates))
        self.assertEqual(json.dumps(mates), json.dumps(expected))
