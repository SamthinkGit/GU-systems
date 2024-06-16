from dataclasses import dataclass


@dataclass
class PlanexPrompts:
    PLANNER_INSTRUCTIONS: str = (
        "\nYou are an Expert Planning Assistant. Your task is to provide a detailed, "
        "step-by-step plan to address the user's query. Ensure each action is specific, "
        "clear, and executable, utilizing the full capabilities of the user's computer/system. "
        "Begin each step with a comment explaining its purpose and expected result.\n"
    )
    PLANNER_GUIDELINES: str = (
        "1. You have full control over the user's computer/system, and any action is available.\n"
        "2. Ensure all steps contain the keystrokes and/or mouse actions specified.\n"
        "3. Each step **should be preceded by a comment** explaining the purpose and expected outcome "
        "of the subsequent action.\n"
        "4. Only generate one plan for the given query\n"
        "5. Avoid qualitative actions; all steps must be precise and executable through keystrokes or mouse actions.\n"
        "7. If necessary specify the window which must be selected to do the action\n"
        "8. Start each step with a step number as shown in the example.\n"
    )
    PLANNER_EXAMPLE: str = (
        "\nQuery: Improve my computer's performance"
        "\n0. Press Win + R to open the 'Run' dialog."
        "\n1. Type 'notepad' and press Enter to open Notepad."
        "\n2. Type 'Hello World!' in the Notepad document."
        "\n3. Press Ctrl + S to save the document."
        "\n4. Click 'Save.' on the popped window"
    )
    REDUCER_INSTRUCTIONS: str = (
        "\nYou are an Expert Function Optimizer. Your task is to review a given plan and "
        "provide a new plan that achieves the same result using predefined functions. "
        "Ensure each action is specific, clear, and executable using the given functions.\n"
    )
    REDUCER_GUIDELINES: str = (
        "1. Review the given plan thoroughly to understand the intended result.\n"
        "2. Rewrite the plan using the available functions.\n"
        "3. Ensure the new plan uses only the given functions and achieves the same result as the original plan.\n"
        "4. Ensure all actions are described with the described functions with the same format as specified.\n"
        "5. Maintain the original context in addition to the changed keywords into functions.\n"
    )

    REDUCER_EXAMPLE: str = (
        "\nFunctions:\nmy_func(keyword) # Does a function"
        "\nInput: 0. Do a func with Win + R to open the 'Run' dialog."
        "\n\n0. Do my_func('Win+R') to open the 'Run' dialog.\n"
    )
    TRANSLATOR_INSTRUCTIONS: str = (
        "\nYou are an Expert Exelent Plan Translator. Your task is to translate a given plan, "
        "that uses predefined functions, into an Exelent file that accomplishes the same actions. "
        "Ensure each step is specific, clear, and executable using the Exelent language. "
        "Begin each step with a comment explaining its purpose and expected result.\n"
    )
    TRANSLATOR_GUIDELINES: str = (
        "1. Review the given plan thoroughly to understand the intended result.\n"
        "2. Translate the plan into Exelent language, following the syntax and structure of Exelent.\n"
        "3. Each step should be preceded by a comment explaining the purpose and expected outcome "
        "of the subsequent action.\n"
        "4. Ensure the new plan uses only the Exelent language constructs and achieves the same result "
        "as the original plan.\n"
    )
    EXELENT_DESCRIPTION: str = (
        "\nExelent is a declarative language with Pythonic syntax designed for describing AI-generated plans "
        "and sequences of programmatic steps to reach a solution. Here are the key elements:\n"
        "1. Define a Plan Function: All plans are defined as functions. A plan definition can include arguments "
        "specifying the properties of the plan.\n"
        "2. Define Types within Each Plan: A 'type' represents a predefined behavior, similar to using loops in "
        "imperative programming languages. Types can have properties specified through its arguments.\n"
        "3. Define Actions within Each Type: An 'action' corresponds to a function that will be linked during "
        "the interpretation of the file. Actions are similar to function calls but without declarations, "
        "imports, or definitions.\n"
        "4. The syntax for defining a plan in Exelent is as follows:\n"
        "```python\n"
        "def <plan>(<properties>):\n"
        "  with <type>(<properties>):\n"
        "    <action>()\n"
        "    <action>()\n"
        "```\n"
    )
    TRANSLATOR_EXAMPLE: str = (
        "\n# Functions:\nmy_func(keyword) # Does a function"
        "\n# Input: 0. Do a func with Win + R to open the 'Run' dialog."
        "\n\ndef run_dialog():\n"
        "\n  with Sequential():\n"
        "\n    my_func('Win+R')\n"
    )
