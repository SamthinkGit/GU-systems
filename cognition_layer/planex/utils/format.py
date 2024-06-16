from langchain.tools import StructuredTool


def format_tool(tool: StructuredTool) -> str:
    args_formatted = [f"{var}: {info['type']}" for var, info in tool.args.items()]
    return f"{tool.name}({','.join(args_formatted)}) # {tool.description}"


def extract_python_code(text: str) -> str:
    tag = "```python"
    start = text.find(tag) + len(tag)
    end = text.rfind("```")
    if start != -1 and end != -1 and start < end:
        return text[start:end]
    return None
