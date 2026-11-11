from .bridge import T1PpowalkGuiBridge
from .web import run_server


def main(args=None):
    run_server(T1PpowalkGuiBridge, "t1_ppowalk_gui", args)


if __name__ == "__main__":
    main()
