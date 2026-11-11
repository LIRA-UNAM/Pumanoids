from .bridge import K1PpowalkGuiBridge
from .web import run_server


def main(args=None):
    run_server(K1PpowalkGuiBridge, "k1_ppowalk_gui", args)


if __name__ == "__main__":
    main()
