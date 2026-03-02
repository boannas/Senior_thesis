from hormone import BiologicalProcessSystem, HormoneSystem, MotivationState
from utilss import KeyboardInput, Fig4RealtimePlotter, simulation_loop
import threading

def main():
    print("=" * 70)
    print("Controls:")
    print("  [U] toggle presence (continuous)")
    print("  hold [C] caress (continuous while held, plus 1-step latch)")
    print("  [H] hit spike | [A] correct spike | [W] wrong spike")
    print("  [Q] quit")
    print("=" * 70)

    bp = BiologicalProcessSystem()  # biological process system
    ms = MotivationState(physio=bp.state, deficit=bp.deficits)
     # motivation state
    hs = HormoneSystem(PB=80, history_size=2400, bp=bp, MS=ms)        # initial hormone system
    kb = KeyboardInput()    # keyboard input handler
    kb.start()

    # start simulation loop in a separate thread
    th = threading.Thread(target=simulation_loop, args=(hs, kb, 0.5), daemon=True)
    th.start()

    plotter = Fig4RealtimePlotter(hs, window_s=60)
    plotter.start()

    kb.running = False
    print("[END]")


if __name__ == "__main__":
    main()
