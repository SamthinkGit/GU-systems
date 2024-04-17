from gusysalb.alb import ALB
from gusysros.tools.feedback import Feedback


def main():
    alb = ALB()
    alb.build_all()
    feedback = Feedback("my_task")
    msg = "Hello Feedback!"

    for _ in range(3):
        print("Mock Feedback published")
        feedback.publish(msg)
    alb.cleanup()

    # TODO: Auto-check if message is correct
    # Actually you can use 'ros2 topic echo /feedback'


if __name__ == '__main__':
    main()
