from pathlib import Path

import yaml
from langchain.docstore.document import Document
from langchain_core.vectorstores import InMemoryVectorStore
from langchain_openai import OpenAIEmbeddings

from ecm.shared import get_root_path


def get_documents_from_path(path: Path) -> list[Document]:
    with open(path) as file:
        file = yaml.safe_load(file)

    docs = []
    for doc in file["docs"]:
        docs.append(
            Document(
                page_content=doc["content"],
                metadata={
                    "title": doc["title"],
                    "experts": doc["requires"]["experts"],
                    "packages": doc["requires"]["packages"],
                    "source": "local",
                },
            )
        )
    return docs


def get_batch_requirements(docs: list[Document]) -> dict[str, list[str]]:
    experts = set()
    packages = set()
    for doc in docs:
        experts.update(doc.metadata["experts"])
        packages.update(doc.metadata["packages"])

    return {
        "experts": list(experts),
        "packages": list(packages),
    }


def generate_inmemory_vectorstore(
    name: str,
    docs: list[Document],
    additional_metadata: dict[str, str] = {},
    embeddings=OpenAIEmbeddings(),
    output_path: Path = get_root_path() / "cognition_layer" / "retrieval" / "stores",
) -> bool:
    requirements = get_batch_requirements(docs)
    requirements.update(additional_metadata)
    store = InMemoryVectorStore(embeddings)
    dir_path = output_path / name

    if dir_path.exists():
        raise FileExistsError(
            f"Directory {dir_path} already exists. Please delete the previous database or use a different name."
        )
    dir_path.mkdir(parents=True, exist_ok=False)

    requirements_path = dir_path / "metadata.yaml"
    db_path = dir_path / "vectorestore.db"

    with open(requirements_path, "w") as file:
        yaml.dump(requirements, file)

    store.add_documents(docs)
    store.dump(db_path)
    return True


def load_vectorstore(
    name: str,
    embeddings=OpenAIEmbeddings(),
    store_path: Path = get_root_path() / "cognition_layer" / "retrieval" / "stores",
) -> InMemoryVectorStore:
    dir_path = store_path / name
    if not dir_path.exists():
        raise FileNotFoundError(f"Directory {dir_path} does not exist.")

    db_path = dir_path / "vectorestore.db"
    store = InMemoryVectorStore.load(db_path, embeddings)
    return store
