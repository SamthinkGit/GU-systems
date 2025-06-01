import json

import requests

from cognition_layer.tools.voice.engine import wav_to_base64

BASE_URL = "http://localhost:8000"


def send_config(params: dict[str, str]):
    """
    Envía un diccionario de configuración al endpoint /config.
    """
    try:
        resp = requests.post(f"{BASE_URL}/config", json={"config": params})
        if resp.status_code == 200:
            print("Configuración enviada correctamente.")
        else:
            print(f"Error al enviar configuración: {resp.status_code} – {resp.text}")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar con la API: {e}")


def send_extra(params: dict[str, str]):
    """
    Envía un diccionario extra (por ejemplo {"audio": "loquesea"}) al endpoint /extra.
    """
    try:
        resp = requests.post(f"{BASE_URL}/extra", json={"extra": params})
        if resp.status_code == 200:
            print("Extra enviado correctamente.")
        else:
            print(f"Error al enviar extra: {resp.status_code} – {resp.text}")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar con la API: {e}")


def send_request(cmd: str):
    """
    Envía uno de los comandos <start>, <pause>, <resume> o <exit> al endpoint /request.
    """
    try:
        resp = requests.post(f"{BASE_URL}/request", json={"request": cmd})
        if resp.status_code == 200:
            print(f"Request '{cmd}' enviado correctamente.")
        else:
            print(f"Error al enviar request: {resp.status_code} – {resp.text}")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar con la API: {e}")


def get_update():
    """
    Llama al endpoint /update y muestra el contenido de 'update'.
    """
    try:
        resp = requests.post(f"{BASE_URL}/update")
        if resp.status_code == 200:
            data = resp.json()
            print("=== Update recibido ===")
            print(json.dumps(data["update"], indent=2, ensure_ascii=False))
        else:
            print(f"Error al pedir update: {resp.status_code} – {resp.text}")
    except Exception as e:
        print(f"[ERROR] No se pudo conectar con la API: {e}")


def print_help():
    print(
        """
Comandos disponibles:

  config key1=value1 key2=value2 ...
      — Envía configuración al endpoint /config.
        Ejemplo: config idioma=es modo=debug

  extra key=value key2=value2 ...
      — Envía datos extra (por ejemplo audio) al endpoint /extra.
        Ejemplo: extra audio=loquesea

  request <start|pause|resume|exit>
      — Envía uno de los valores de InputRequest al endpoint /request.
        Ejemplo: request start

  update
      — Llama a /update y muestra el estado (campo "update") que devuelve el servidor.

  help
      — Muestra esta ayuda.

  quit
      — Sale de la consola.

Nota:
  • Los comandos `config` y `extra` aceptan pares clave=valor separados por espacios.
  • Para `request`, usa exactamente uno de: start, pause, resume, exit.
"""
    )


def parse_key_value_pairs(tokens: list[str]) -> dict[str, str]:
    """
    Dado un listado de tokens en formato "clave=valor", devuelve un diccionario.
    Ignora tokens que no tengan '=' o estén vacíos.
    """
    result: dict[str, str] = {}
    for tok in tokens:
        if "=" in tok:
            k, v = tok.split("=", 1)
            k = k.strip()
            v = v.strip()
            if k != "":
                result[k] = wav_to_base64(v)  # Since we are sending wavs
    return result


def console_loop():
    print("=== Consola de interacción con NovaAPI ===")
    print("Escribe 'help' para ver los comandos disponibles.\n")
    while True:
        try:
            line = input("> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nSaliendo de la consola.")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd == "help":
            print_help()

        elif cmd == "quit":
            print("Saliendo de la consola.")
            break

        elif cmd == "config":
            # Reste de tokens son pares clave=valor
            if len(parts) < 2:
                print(
                    "Debe especificar al menos un par clave=valor. Escribe 'help' para más info."
                )
                continue
            kv = parse_key_value_pairs(parts[1:])
            if kv:
                send_config(kv)
            else:
                print("No se encontraron pares válidos clave=valor.")

        elif cmd == "extra":
            # Reste de tokens son pares clave=valor; normalmente mínimo "audio=..."
            if len(parts) < 2:
                print(
                    "Debe especificar al menos un par clave=valor (por ejemplo audio=loquesea)."
                )
                continue
            kv = parse_key_value_pairs(parts[1:])
            if kv:
                send_extra(kv)
            else:
                print("No se encontraron pares válidos clave=valor.")

        elif cmd == "request":
            # Esperamos exactamente uno de: start, pause, resume, exit
            if len(parts) != 2:
                print("Sintaxis: request <start|pause|resume|exit>")
                continue
            action = parts[1].lower()
            mapping = {
                "start": "<start>",
                "pause": "<pause>",
                "resume": "<resume>",
                "exit": "<exit>",
            }
            if action in mapping:
                send_request(mapping[action])
            else:
                print("Valor inválido. Usar uno de: start, pause, resume, exit.")

        elif cmd == "update":
            get_update()

        else:
            print(
                f"Comando desconocido: '{cmd}'. Escribe 'help' para ver la lista de comandos."
            )


if __name__ == "__main__":
    console_loop()
