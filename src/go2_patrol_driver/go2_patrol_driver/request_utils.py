import json

try:
    from unitree_api.msg import Request
except ImportError:  # Allows pure logic tests on a non-ROS machine.
    Request = None


def sport_request(api_id: int, parameter=None, counter: int = 0) -> Request:
    """Build a Unitree sport request using the official ROS 2 message schema."""
    if Request is not None:
        request = Request()
        request.header.identity.api_id = int(api_id)
        request.header.identity.id = int(counter)
        request.header.lease.id = 0
        request.header.policy.noreply = True
    else:
        request = {
            "identity": {"api_id": int(api_id), "id": int(counter)},
            "parameter": parameter,
        }
    if parameter is not None:
        serialized = json.dumps(parameter) if not isinstance(parameter, str) else parameter
        if isinstance(request, dict):
            request["serialized_parameter"] = serialized
        else:
            request.parameter = serialized
    return request
