import multiprocessing
import os
import sys


def sync_callback():
    from ecm_communications.bootstraps.autodiscover_pair import autodiscover

    print("[Callback] Running autodiscover...")
    success = autodiscover(role="client", timeout=3)
    if not success:
        print("[Callback] Error: No peer found.")
        return "Nova no se ha encontrado. Asegúrate de estar en el mismo Wi-Fi."
    return "¡Sincronización exitosa con Nova!"


def run_listener(peer_ip):
    # Imports dentro de la función, sólo en el proceso hijo
    from ecm.tools.registry import ItemRegistry, Storage
    from ecm_communications.tools.listener import Listener

    # Configuración del Storage y registro de ítems
    Storage("ITEM_REGISTRY_CONFIG")["autoload-static"] = True
    reg = ItemRegistry()
    for item in (
        "screenshot",
        "keyboard",
        "mouse-simple",
        "apps-management",
        "spotlight",
        "request-selection",
        "molmo_element_finder_multiple_actions",
        "simple-read-ocr",
        "return-response",
    ):
        reg.autoload(item)
    reg.summary()

    listener = Listener(peer_ip=peer_ip)
    listener.listen()


def start_callback():
    global listener_p
    print("[Callback] Nova Started")
    peer_ip = os.environ.get("ECM_PEER_IP")
    if not peer_ip:
        print("[Callback] Error: ECM_PEER_IP not set.")
        return
    # Con 'spawn' en Windows, freeze_support() ya evitó el re-arranque de la UI en el hijo
    listener_p = multiprocessing.Process(target=run_listener, args=(peer_ip,))
    listener_p.start()
    return listener_p  # si quieres guardar la referencia globalmente


def toggle_callback(state):
    global localhost
    localhost = state


def exit_callback():
    global listener_p
    print("[Callback] Exiting Nova...")
    if listener_p and listener_p.is_alive():
        listener_p.terminate()
        listener_p.join()
        print("[Callback] Listener stopped.")


def main():
    # Esto evita el "re-boot" de todo cuando spawn lanza un hijo
    multiprocessing.freeze_support()

    # Imports de UI aquí, sólo en el proceso principal
    from PyQt5.QtWidgets import QApplication
    from ecm.tools.ui.client_ui import NovaUI

    app = QApplication(sys.argv)
    # Crea la ventana, conecta callbacks (nota que start_callback ya devuelve el Process)
    window = NovaUI(
        on_sync=sync_callback,
        on_start=start_callback,
        on_toggle_localhost=toggle_callback,
        on_exit=exit_callback,
    )
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
