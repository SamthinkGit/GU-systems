from langchain.tools import StructuredTool


def format_tool(tool: StructuredTool) -> str:
    args_formatted = [f"{var}: {info['type']}" for var, info in tool.args.items()]
    return f"{tool.name}({','.join(args_formatted)}) # {tool.description}"
