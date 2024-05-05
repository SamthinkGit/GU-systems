from execution_layer.rosa.interfaces.alb import ALB
from execution_layer.rosa.ros2 import Feedback


def main():
    alb = ALB()
    alb.build_all()
    feedback = Feedback()
    msg = "Hello Feedback!"

    for _ in range(3):
        print("Mock Feedback published")
        feedback.publish(msg)
    alb.cleanup()

    # TODO: Auto-check if message is correct
    # Actually you can use 'ros2 topic echo /feedback'


if __name__ == "__main__":
    main()
