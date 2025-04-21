from langchain.tools import StructuredTool


def format_tool(tool: StructuredTool) -> str:
    """Given a langchain tool, returns an AI comprehensible representation."""
    args_formatted = [f"{var}: {info['type']}" for var, info in tool.args.items()]
    return f"{tool.name}({','.join(args_formatted)}) # {tool.description}"


def extract_python_code(text: str) -> str:
    """Given a python code in a code block extrads the code inside the block"""
    tag = "```python"
    start = text.find(tag) + len(tag)
    end = text.rfind("```")
    if start != -1 and end != -1 and start < end:
        return text[start:end]
    return None
