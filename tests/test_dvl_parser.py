from cps5_companion.dvl import DvlStreamParser
from cps5_companion.models import DvlPosition


def test_fragmented_and_coalesced_frames_are_recovered() -> None:
    parser = DvlStreamParser()
    assert parser.feed(b'{"type":"position_local","x":1.') == []
    reports = parser.feed(
        b'25,"y":-0.5}\r\n{"type":"status","ok":true}\r\npartial'
    )
    assert len(reports) == 2
    assert DvlPosition.from_report(reports[0]) == DvlPosition(1.25, -0.5)
    assert reports[1]["type"] == "status"


def test_invalid_frame_does_not_discard_following_valid_frame() -> None:
    parser = DvlStreamParser()
    reports = parser.feed(b'not-json\r\n{"type":"position_local","x":2,"y":3}\r\n')
    assert reports == [{"type": "position_local", "x": 2, "y": 3}]
