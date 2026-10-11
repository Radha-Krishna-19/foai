"""Tiny helper: tests record the value they actually measured so the testing document can show it."""
ACTUALS = {}


def record(test_id: str, text: str):
    ACTUALS[test_id] = text
