from cognition_layer.tools.vision.molmo.description2coordinates import (
    parse_multiple_points,
)


def test_molmo_points_are_being_parsed():
    points = """
    <point x="61.5" y="47.7" alt="Mow">Mow</point>
    <point x="62.5" y="47.7" alt="Miaw">Miaw</point>
    """
    points: dict = parse_multiple_points(points)
    assert len(points) == 2
    assert points[0]["x"] == 61.5
    assert points[0]["y"] == 47.7
    assert points[0]["alt"] == "Mow"
    assert points[0]["text"] == "Mow"


def test_malformed_points_are_ignored():
    points = """
    <point x=61.5 y=47.7 alt=''>Mow</point>
    <point x="62.5" y="47.7" alt="Miaw"Miaw/point>
    <point x="62.5 y="47.7" alt="Miaw">Miaw</point>
    """
    points: dict = parse_multiple_points(points)
    assert len(points) == 0
