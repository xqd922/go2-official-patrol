from go2_patrol_driver.request_utils import sport_request
from go2_patrol_driver.sport_api_ids import ROBOT_SPORT_API_ID_MOVE


def test_sport_request_uses_official_api_id_and_json_parameter():
    request = sport_request(ROBOT_SPORT_API_ID_MOVE, {"x": 0.1, "y": 0.0, "z": 0.2}, 42)
    if hasattr(request, "header"):
        assert request.header.identity.api_id == 1008
        assert request.header.identity.id == 42
        assert '"x": 0.1' in request.parameter
    else:
        assert request["identity"]["api_id"] == 1008
        assert request["identity"]["id"] == 42
        assert '"x": 0.1' in request["serialized_parameter"]
