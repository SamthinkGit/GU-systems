import json
import os
import stat

from cryptography.fernet import Fernet

from ecm.shared import get_root_path


class PersistentConfig:
    """
    Class to manage persistent key-value configurations in a fixed file,
    with support for secure (encrypted) fields and default values.
    Does not use instances; all operations are via classmethods.
    """

    _config_path = str(get_root_path() / ".persistent_config.json")
    _key_path = str(get_root_path() / ".lock")

    # Internal dictionary to store default values for fields
    _defaults = {}

    @classmethod
    def _ensure_paths_exist(cls):
        """
        Ensure that the parent directory of the config file exists.
        """
        folder = os.path.dirname(cls._config_path)
        if folder and not os.path.isdir(folder):
            os.makedirs(folder, exist_ok=True)

    @classmethod
    def _load_config(cls):
        """
        Load the configuration dictionary from the JSON file.
        If the file does not exist or is invalid, return an empty dict.
        """
        cls._ensure_paths_exist()
        if not os.path.isfile(cls._config_path):
            return {}
        with open(cls._config_path, encoding="utf-8") as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                # If the JSON is corrupted, discard it and start fresh
                return {}
        return data

    @classmethod
    def _save_config(cls, data: dict):
        """
        Save the configuration dictionary to the JSON file,
        setting secure permissions (600) so only the current user can read/write.
        """
        cls._ensure_paths_exist()
        with open(cls._config_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        # Permissions 0o600 (read/write only for the owner)
        os.chmod(cls._config_path, stat.S_IRUSR | stat.S_IWUSR)

    @classmethod
    def _get_encryption_key(cls) -> bytes:
        """
        Obtain (or generate, if it doesn't exist) a Fernet key for encryption/decryption.
        The key is stored in _key_path with permissions 600.
        """
        if not os.path.isfile(cls._key_path):
            # Generate a new key and save it
            key = Fernet.generate_key()
            # Ensure the directory exists
            folder = os.path.dirname(cls._key_path)
            if folder and not os.path.isdir(folder):
                os.makedirs(folder, exist_ok=True)
            with open(cls._key_path, "wb") as f_key:
                f_key.write(key)
            os.chmod(cls._key_path, stat.S_IRUSR | stat.S_IWUSR)
            return key

        # If it already exists, just read it
        with open(cls._key_path, "rb") as f_key:
            key = f_key.read()
        return key

    @classmethod
    def set(cls, key: str, value):
        """
        Save a key-value pair in the config file.
        If the key already exists, overwrite it; if not, create it.
        """
        cfg = cls._load_config()
        cfg[key] = value
        cls._save_config(cfg)

    @classmethod
    def create(cls, key: str, default_value, replace: bool = False):
        """
        Create a field (key) with a default value.
        - If replace is False and the key already exists, do nothing.
        - If replace is True, overwrite the existing value.
        Also stores the default in _defaults for later reset.
        """
        cls._defaults[key] = default_value
        cfg = cls._load_config()

        if key not in cfg or replace:
            cfg[key] = default_value
            cls._save_config(cfg)

    @classmethod
    def get(cls, key: str, default_value=None):
        """
        Get the value associated with 'key'. If it doesn't exist, return default_value.
        If the key corresponds to a secure field, decrypt before returning.
        """
        cfg = cls._load_config()
        if key not in cfg:
            return default_value

        val = cfg[key]
        # Detect if it is a secure field (structure {'_secure': True, 'value': <token>})
        if isinstance(val, dict) and val.get("_secure") is True:
            token = val.get("value", "").encode("utf-8")
            f = Fernet(cls._get_encryption_key())
            try:
                plaintext = f.decrypt(token).decode("utf-8")
            except Exception:
                # If decryption fails (corruption or key changed), return None
                return None
            return plaintext

        return val

    @classmethod
    def create_secure(cls, key: str, value: str):
        """
        Create or overwrite a secure entry (encrypt the value using Fernet).
        The value is stored as:
            { "_secure": True, "value": <base64_token> }
        """
        key_bytes = cls._get_encryption_key()
        f = Fernet(key_bytes)
        token = f.encrypt(value.encode("utf-8")).decode("utf-8")

        cfg = cls._load_config()
        cfg[key] = {"_secure": True, "value": token}
        cls._save_config(cfg)

    @classmethod
    def reset(cls):
        """
        Reset all keys in the config file to their default values
        (as registered in _defaults). Creates a new dict with all defaults
        and writes it out.
        """
        new_cfg = {}
        for key, def_val in cls._defaults.items():
            new_cfg[key] = def_val
        cls._save_config(new_cfg)
