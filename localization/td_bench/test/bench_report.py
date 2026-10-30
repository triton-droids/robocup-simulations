"""Log measured values next to their pass conditions (read by test_report/make_results.py via junitxml)."""


def show(record_property, what: str, measured: str, expected: str) -> None:
    record_property(what, f"{measured} | {expected}")
