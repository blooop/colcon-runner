from jazzy_py_demo import compose_greeting


def test_compose_greeting_strips_whitespace():
    assert compose_greeting("  Jazzy  ") == "Hello, Jazzy! Welcome to ROS Jazzy."


def test_compose_greeting_defaults_to_world():
    assert compose_greeting("   ") == "Hello, world! Welcome to ROS Jazzy."
