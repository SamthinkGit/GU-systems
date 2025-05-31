# This installer is used for the client installation,
# For installing a complete ECM, please refer to the documentation
# and use the scripts at the directory /installer
from setuptools import find_namespace_packages
from setuptools import setup

setup(
    name="gu_systems",
    version="0.1.0",
    description="GU-systems core package",
    author="Sebastian Mayorquín",
    packages=find_namespace_packages(
        include=[
            "action_space",
            "action_space.*",
            "cognition_layer",
            "cognition_layer.*",
            "execution_layer",
            "execution_layer.*",
            "ecm",
            "ecm.*",
        ]
    ),
    include_package_data=True,
    install_requires=[
        "grpcio",
        "langchain-core",
        "langchain-openai",
        "requests",
        "colorlog",
        "dotenv",
        "colorama",
        "protobuf",
        "pillow",
        "pyautogui",
        "psutil",
        "pyqt5",
        "pydub",
        "replicate",
        "opencv-python",
        "rapidocr",
        "elevenlabs",
        "playsound",
        "open-interpreter"
        "markdown"
        "PyQtWebEngine"
        "pyaudio"
        # "ecm_communications*", Private repo on /external
    ],
    python_requires=">=3.8",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    namespace_packages=[
        "action_space",
        "cognition_layer",
        "execution_layer",
        "ecm",
    ],
)
