"""
Task Management and Sequencing
==============================

This module defines and manages different types of sequences for tasks in a system. Here you can implement
new protocols for ordering a sequence of actions, expanding the Action Space B (where SeqType(B) => A').

It supports dynamic registration of sequence types, and enforces a structure where each sequence type can
manage its execution and error handling.
"""
import random
from enum import Enum
from typing import Callable
from typing import Dict
from typing import Type

from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.feedback import Feedback
from execution_layer.rosa.ros2.tools.packages import SequencePackage


class ReservedTypeCode(Enum):

    UNDEFINED = -1
    SOFT_STOP = -2
    HARD_STOP = -3


class SequenceType:
    """
    Base class for defining a Sequence Type (or BT Node) in the system.
    :param pkg: The SequencePackage associated with this sequence type.
    """

    type_code = None
    _registry: Dict[int, Type["SequenceType"]] = {}

    def __init__(self, pkg: SequencePackage) -> None:
        """
        Initializes a SequenceType instance with a package, ensures subclassing.
        """
        if self.__class__ == SequenceType:
            raise TypeError(
                "SequenceType cannot be instantiated directly, please use a subclass"
            )

        assert (
            pkg.type == self.type_code
        ), "Package type does not match the expected type"

        self.pkg = pkg
        self._soft_stop = False
        self.exit: Callable = None
        self.feedback = Feedback()

    @classmethod
    def register_type(
        cls, type_code: int, type_class: Type["SequenceType"], force: bool = False
    ):
        """
        Registers a sequence type class under a specific type code.
        :note: Each new SequenceType should use this function to be avaliable for the user. Generally
        this is automatically made by the ALB if the file is in the appropiate paths

        :param type_code: The unique code identifying the sequence type.
        :param type_class: The class to register as handling this type code.
        :param force: Whether to force registration even if a type with the same code exists.
        """

        if type_code in cls._registry:

            if not force and cls._registry[type_code] != type_class:
                raise TypeError(
                    f"SequenceType with type_code {type_code} has already been registered"
                )

        cls._registry[type_code] = type_class

    @classmethod
    def auto_register_type(cls, type: Type["SequenceType"]):
        """
        [MANDATORY] Automatically registers a sequence type class using its defined type code.
        IMPORTANT: Ensure to register your SequenceType at the end of the node, or by
        subscribing the file to the ALB
        """
        cls.register_type(type_code=type.get_type(), type_class=type)

    @classmethod
    def from_pkg(cls, pkg: SequencePackage) -> "SequenceType":
        """
        Instantiates a SequenceType from a package, using the registered class for its type.
        After the instantiation, you should be able to use .run() method

        :param pkg: The package containing data to instantiate the SequenceType.
        :return: An instance of a subclass of SequenceType.
        """

        type_class = cls._registry.get(pkg.type)
        if type_class is None:
            raise ValueError(
                f"No registered class for package type: {pkg.type}. Have you used auto_register_type()?"
            )
        return type_class(pkg)

    @classmethod
    def get_type(cls) -> int:
        """
        Returns the type code associated with this SequenceType.
        """
        return cls.type_code

    def soft_stop(self) -> None:
        """Sets the SOFT-STOP variable to true. It will stop whenever is possible and clean"""
        self._soft_stop = True

    def soft_stop_called(self) -> None:
        return self._soft_stop

    def run(self) -> None:
        raise NotImplementedError(
            "SequenceType cannot be instantiated directly, please use a subclass"
        )

    def at_exit(self, func: Callable) -> None:
        """Define a function to be executed after the SequenceType has finished all the execution"""
        self.exit = func


class SimpleSequence(SequenceType):
    """
    A simple sequence type that processes a list of actions sequentially.
    :param pkg: The SequencePackage associated with this sequence.
    """

    _logger = get_logger("SimpleSequence")
    type_code = 5

    def __init__(self, pkg: SequencePackage) -> None:
        super().__init__(pkg)

    def run(self):
        """Runs all the functions in the package sequentially"""
        success = True
        self._logger.debug(f"Starting to run: {self.pkg.task_id}")
        for action in self.pkg.actions:
            try:

                if self.soft_stop_called():
                    self.feedback.publish(
                        "Abort (By Soft-Stop)", _status=ExecutionStatus.ABORT
                    )
                    success = False
                    break

                func = ItemRegistry.get_function(action.action_id)
                args = action.args
                kwargs = action.kwargs
                func(*args, **kwargs)
                self.feedback.publish("Step", _status=ExecutionStatus.STEP)

            except Exception:
                self._logger.error(
                    f"Exception when running {action} in task: {self.pkg.task_id}",
                    exc_info=True,
                )
                success = False

        if success:
            self.feedback.publish("Success", _status=ExecutionStatus.SUCCESS)
        else:
            self.feedback.publish("Abort (Failed)", _status=ExecutionStatus.ABORT)
        self._logger.debug(f"Execution Finished: {self.pkg.task_id}")
        self.feedback.publish("Finish", _status=ExecutionStatus.FINISH)

        if self.exit is not None:
            self.exit(self.pkg.task_id)


class ControlledSequence(SequenceType):
    """
    A human-in-the-loop based sequence. It runs the obtained steps sequentially,
    however, it waits for a signal to continue before executing each step.
    """

    _logger = get_logger("ControlledSequence")
    type_code = 9

    def __init__(self, pkg: SequencePackage) -> None:
        super().__init__(pkg)

    def run(self):
        """Runs all the functions in the package sequentially"""
        success = True
        self._logger.debug(f"Starting to run: {self.pkg.task_id}")
        result = None
        for action in self.pkg.actions:
            try:

                # Managing Soft-Stop
                if self.soft_stop_called():
                    self.feedback.publish(
                        "Abort (By Soft-Stop)", _status=ExecutionStatus.ABORT
                    )
                    success = False
                    break

                # Requesting Approval
                response_code = str(random.randint(1000, 9999))

                self.feedback.publish(
                    object=[result, response_code], _status=ExecutionStatus.REQUEST_TO_CONTINUE
                )
                self._logger.debug("Waiting for approval to execute action.")
                response = self.feedback.wait_for_response(
                    response_code=response_code
                )

                if response._exec_status == ExecutionStatus.ABORT:
                    self._logger.debug(f"Abort (By response) on task: {self.pkg.task_id}")
                    success = False
                    break
                self._logger.debug("Approval to continue received.")

                # Calling Function
                func = ItemRegistry.get_function(action.action_id)
                args = action.args
                kwargs = action.kwargs
                result = func(*args, **kwargs)
                self.feedback.publish("Step", _status=ExecutionStatus.STEP)

            except Exception:
                self._logger.error(
                    f"Exception when running {action} in task: {self.pkg.task_id}",
                    exc_info=True,
                )
                success = False

        if success:
            self.feedback.publish("Success", _status=ExecutionStatus.SUCCESS)
        else:
            self.feedback.publish("Abort (Failed)", _status=ExecutionStatus.ABORT)

        self._logger.debug(f"Execution Finished: {self.pkg.task_id}")
        self.feedback.publish("Finish", _status=ExecutionStatus.FINISH)

        if self.exit is not None:
            self.exit(self.pkg.task_id)


# ----- YOU CAN ADD HERE THE TYPES THAT SHOULD AUTOREGISTER WHEN IMPORTING THIS FILE (ALB) --------
SequenceType.auto_register_type(SimpleSequence)
SequenceType.auto_register_type(ControlledSequence)

# -------------------------------------------------------------------------------------------------
