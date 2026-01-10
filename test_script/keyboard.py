from pynput import keyboard as pynput_keyboard
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

class StimulusTable:
    def __init__(self):
        # All stimuli are discrete events
        self.table = {
            "HIT": [
                {"target": 'AVP', "type": 'hormone', 'alpha': 0.0075, 'beta': 1.0}
            ],
            "CARESS": [
                {"target": "OT", "type": "hormone", "alpha": 0.0075, "beta": "0.02 * PB"},
            ],
            "CORRECT": [
                {"target": "OT", "type": "hormone", "alpha": 0.001, "beta": "0.02 * PB"},
                {"target": "DA", "type": "hormone", "alpha": 0.005, "beta": 1.0}
            ],
            "WRONG": [
                {"target": "AVP", "type": "hormone", "alpha": 0.002, "beta": 1.0}
            ],
            "USER_PRESENCE": [
                {"target": "AVP", "type": "hormone", "alpha": 0.003, "beta": 1.0},
                {"target": "OT", "type": "hormone", "alpha": 0.003, "beta": "0.02 * PB"},
                {"target": "DA", "type": "hormone", "alpha": 0.004, "beta": 1.0}
            ]
        }

    def get_effects(self, stimulus):
        return self.table.get(stimulus, [])


class HormoneSystem:
    def __init__(self, PB=50):
        self.PB = PB
        self.time = 0
        
        # Hormone secretion rates - start at circadian baseline
        cr_DA, cr_OT, cr_AVP = self._initial_circadian()
        self.DA = cr_DA
        self.OT = cr_OT
        self.AVP = cr_AVP
        
        self.OT_auto = 0.001
        
        # User presence state affects baseline
        self.user_present = False
        self.baseline_boost = 0.0  # Boost to baseline when user is present
        
        # History for plotting
        self.history_size = 200
        self.time_history = deque(maxlen=self.history_size)
        self.DA_history = deque(maxlen=self.history_size)
        self.OT_history = deque(maxlen=self.history_size)
        self.AVP_history = deque(maxlen=self.history_size)
        self.PB_history = deque(maxlen=self.history_size)
        
        # Stimulus markers for discrete events
        self.stimulus_events = deque(maxlen=50)
        
        self.elapsed_time = 0
        self.stimulus_table = StimulusTable()
        
    def _initial_circadian(self):
        """Get initial circadian rhythm values"""
        cr_DA = 0.05 + 0.3 * math.cos(2 * math.pi * ((0 - 20) / 24))
        cr_OT = 0.05 + 0.3 * math.cos(2 * math.pi * ((0 - 10) / 24))
        cr_AVP = 0.05 + 0.3 * math.cos(2 * math.pi * ((0 - 10) / 24))
        return cr_DA, cr_OT, cr_AVP
        
    def circadian_rhythm(self, t):
        """Calculate circadian rhythms for each hormone (Table 1)"""
        cr_DA = 0.05 + 0.3 * math.cos(2 * math.pi * ((t - 20) / 24))
        cr_OT = 0.05 + 0.3 * math.cos(2 * math.pi * ((t - 10) / 24))
        cr_AVP = 0.05 + 0.3 * math.cos(2 * math.pi * ((t - 10) / 24))
        return cr_DA, cr_OT, cr_AVP
    
    def eval_beta(self, beta_value):
        """Evaluate beta expression (Eq. 3 from paper)"""
        if isinstance(beta_value, (int, float)):
            return float(beta_value)
        if isinstance(beta_value, str):
            allowed = {"PB": self.PB}
            try:
                return float(eval(beta_value, {"__builtins__": {}}, allowed))
            except Exception as e:
                return 1.0
        return 1.0
    
    def apply_stimulus(self, stimulus_name, intensity=100):
        """Apply stimulus effect using Eq. (3): se_k(t) = α_k * β_k * si_k(t)"""
        effects = self.stimulus_table.get_effects(stimulus_name)
        
        print(f"\n[STIMULUS] {stimulus_name} @ t={self.elapsed_time:.1f}s")
        
        for effect in effects:
            alpha = effect['alpha']
            beta = self.eval_beta(effect['beta'])
            stimulus_effect = alpha * beta * intensity  # Equation (3)
            
            target = effect['target']
            
            if target == 'DA':
                self.DA = max(0.01, min(1.0, self.DA + stimulus_effect))
            elif target == 'OT':
                self.OT = max(0.01, min(1.0, self.OT + stimulus_effect))
            elif target == 'AVP':
                self.AVP = max(0.01, min(1.0, self.AVP + stimulus_effect))
            
            print(f"  {target}: {stimulus_effect:+.4f}")
        
        # Record discrete stimulus events for plotting
        y_pos = 0.9 - (len(self.stimulus_events) % 3) * 0.1
        self.stimulus_events.append((self.elapsed_time, stimulus_name, y_pos))
    
    def set_user_presence(self, present):
        """Toggle user presence - affects baseline only"""
        if present and not self.user_present:
            # User just appeared - apply one-time stimulus
            print(f"\n{'='*50}")
            print(f"[USER ENTERS] Applying presence stimulus")
            print(f"{'='*50}")
            self.apply_stimulus("USER_PRESENCE", intensity=100)
            self.user_present = True
            self.baseline_boost = 0.05  # Small boost to baseline
        elif not present and self.user_present:
            # User just left
            print(f"\n{'='*50}")
            print(f"[USER LEAVES] Removing presence effect")
            print(f"{'='*50}")
            self.user_present = False
            self.baseline_boost = 0.0
    
    def update(self, dt=0.5):
        """Update hormone levels with circadian rhythm and decay"""
        cr_DA, cr_OT, cr_AVP = self.circadian_rhythm(self.time)
        
        # Add baseline boost if user is present
        baseline_DA = cr_DA + self.baseline_boost
        baseline_OT = cr_OT + self.baseline_boost
        baseline_AVP = cr_AVP + self.baseline_boost
        
        # Decay towards (possibly boosted) baseline
        decay_rate = 0.95 #0.95
        self.DA = max(0.01, min(1.0, self.DA * decay_rate + baseline_DA * (1 - decay_rate)))
        self.OT = max(0.01, min(1.0, self.OT * decay_rate + baseline_OT * (1 - decay_rate)))
        self.AVP = max(0.01, min(1.0, self.AVP * decay_rate + baseline_AVP * (1 - decay_rate)))
        
        # Oxytocin auto-stimulation
        if self.OT > 0.2:
            self.OT = min(1.0, self.OT + self.OT_auto * 0.5)
        
        self.time = (self.time + dt / 3600) % 24
        self.elapsed_time += dt
    
    def update_PB(self, dt=0.5):
        """Update pair-bonding value using Eq. (1)"""
        dr = 0.01  # decay rate
        delta_PB = 0.2 * self.OT - 0.2 * self.AVP - dr
        self.PB = max(0, min(100, self.PB + delta_PB * dt))
    
    def record_history(self):
        """Record current state for plotting"""
        self.time_history.append(self.elapsed_time)
        self.DA_history.append(self.DA)
        self.OT_history.append(self.OT)
        self.AVP_history.append(self.AVP)
        self.PB_history.append(self.PB)


class KeyboardInput:
    def __init__(self):
        self.key_map = {
            "h": "HIT",
            "c": "CARESS",
            "a": "CORRECT",
            "w": "WRONG",
            "u": "USER_PRESENCE_TOGGLE",
            "q": "QUIT"
        }
        self.last_stimulus = None
        self.running = True
        self.user_present = False  # Toggle state

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return

        if k in self.key_map:
            action = self.key_map[k]
            
            if action == "USER_PRESENCE_TOGGLE":
                self.user_present = not self.user_present
                status = "PRESENT ✓" if self.user_present else "ABSENT ✗"
                print(f"\n{'='*50}")
                print(f"[TOGGLE] User is now {status}")
                print(f"{'='*50}")
            elif action == "QUIT":
                self.running = False
                return False
            else:
                self.last_stimulus = action

    def start(self):
        listener = pynput_keyboard.Listener(on_press=self.on_press)
        listener.start()


class RealtimePlotter:
    def __init__(self, hormone_system, kb_input):
        self.hormone_system = hormone_system
        self.kb_input = kb_input
        
        # Create figure with 2 subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(14, 9))
        self.fig.suptitle('Real-time Neuroendocrine Hormone Simulation\n(Based on Maroto-Gómez et al., 2024)', 
                         fontsize=14, fontweight='bold')
        
        # Initialize lines for hormone plot
        self.line_DA, = self.ax1.plot([], [], 'b-', label='Dopamine (DA)', linewidth=2.5, alpha=0.8)
        self.line_OT, = self.ax1.plot([], [], 'g-', label='Oxytocin (OT)', linewidth=2.5, alpha=0.8)
        self.line_AVP, = self.ax1.plot([], [], 'r-', label='Arginine Vasopressin (AVP)', linewidth=2.5, alpha=0.8)
        
        # Initialize line for PB plot
        self.line_PB, = self.ax2.plot([], [], 'm-', label='Pair-Bonding', linewidth=2.5, alpha=0.8)
        
        # Setup hormone plot
        self.ax1.set_xlabel('Time (seconds)', fontsize=11)
        self.ax1.set_ylabel('Hormone Secretion Rate (0-1)', fontsize=11)
        self.ax1.set_ylim(0, 1.1)
        self.ax1.legend(loc='upper left', fontsize=10)
        self.ax1.grid(True, alpha=0.3, linestyle='--')
        
        # Setup PB plot
        self.ax2.set_xlabel('Time (seconds)', fontsize=11)
        self.ax2.set_ylabel('Pair-Bonding (0-100)', fontsize=11)
        self.ax2.set_ylim(0, 105)
        self.ax2.legend(loc='upper left', fontsize=10)
        self.ax2.grid(True, alpha=0.3, linestyle='--')
        
        # Stimulus annotations
        self.stimulus_annotations = []
        self.presence_indicator = None
        
        # Color map for stimuli
        self.stimulus_colors = {
            'HIT': 'red',
            'CARESS': 'green',
            'CORRECT': 'blue',
            'WRONG': 'orange',
        }
        
        plt.tight_layout()
    
    def init_plot(self):
        self.line_DA.set_data([], [])
        self.line_OT.set_data([], [])
        self.line_AVP.set_data([], [])
        self.line_PB.set_data([], [])
        return self.line_DA, self.line_OT, self.line_AVP, self.line_PB
    
    def update_plot(self, frame):
        if len(self.hormone_system.time_history) > 0:
            times = list(self.hormone_system.time_history)
            
            # Update hormone lines
            self.line_DA.set_data(times, list(self.hormone_system.DA_history))
            self.line_OT.set_data(times, list(self.hormone_system.OT_history))
            self.line_AVP.set_data(times, list(self.hormone_system.AVP_history))
            
            # Update PB line
            self.line_PB.set_data(times, list(self.hormone_system.PB_history))
            
            # Update x-axis limits with some padding
            if len(times) > 1:
                x_max = times[-1] + 5
                x_min = max(0, times[-1] - 60)  # Show last 60 seconds
                self.ax1.set_xlim(x_min, x_max)
                self.ax2.set_xlim(x_min, x_max)
            
            # Clear old annotations
            for ann in self.stimulus_annotations:
                try:
                    ann.remove()
                except:
                    pass
            self.stimulus_annotations.clear()
            
            # Show user presence as background shading
            if self.kb_input.user_present and len(times) > 0:
                x_min_plot = max(0, times[-1] - 60)
                x_max_plot = times[-1] + 5
                
                # Add purple background shading to both plots
                shade1 = self.ax1.axvspan(x_min_plot, x_max_plot, alpha=0.15, color='purple', zorder=0)
                shade2 = self.ax2.axvspan(x_min_plot, x_max_plot, alpha=0.15, color='purple', zorder=0)
                self.stimulus_annotations.extend([shade1, shade2])
                
                # Add "USER PRESENT" indicator
                indicator = self.ax1.text(0.98, 0.97, '👤 USER PRESENT', 
                                        transform=self.ax1.transAxes,
                                        fontsize=11, color='white', fontweight='bold',
                                        ha='right', va='top',
                                        bbox=dict(boxstyle='round,pad=0.6', facecolor='purple', 
                                                edgecolor='darkviolet', linewidth=2, alpha=0.9))
                self.stimulus_annotations.append(indicator)
            
            # Add discrete stimulus event markers
            # Create a snapshot to avoid "deque mutated during iteration" error
            stimulus_events_snapshot = list(self.hormone_system.stimulus_events)
            for event_time, stim_name, y_pos in stimulus_events_snapshot:
                if event_time >= max(0, times[-1] - 60):
                    color = self.stimulus_colors.get(stim_name, 'gray')
                    
                    # Vertical line on hormone plot
                    line1 = self.ax1.axvline(x=event_time, color=color, linestyle='--', 
                                            alpha=0.6, linewidth=1.5, zorder=1)
                    self.stimulus_annotations.append(line1)
                    
                    # Text annotation
                    ann = self.ax1.text(event_time, y_pos, stim_name[:3], 
                                       fontsize=9, color=color, fontweight='bold',
                                       ha='center', va='center',
                                       bbox=dict(boxstyle='round,pad=0.4', facecolor='white', 
                                               edgecolor=color, linewidth=2, alpha=0.85),
                                       zorder=2)
                    self.stimulus_annotations.append(ann)
                    
                    # Vertical line on PB plot
                    line2 = self.ax2.axvline(x=event_time, color=color, linestyle='--', 
                                            alpha=0.6, linewidth=1.5, zorder=1)
                    self.stimulus_annotations.append(line2)
        
        return self.line_DA, self.line_OT, self.line_AVP, self.line_PB
    
    def start(self):
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot,
                                interval=100, blit=False, cache_frame_data=False)
        plt.show()


def simulation_loop(hormone_system, kb_input):
    """Background thread for simulation updates"""
    dt = 0.5
    
    while kb_input.running:
        # Handle discrete stimuli (one-time events)
        if kb_input.last_stimulus:
            hormone_system.apply_stimulus(kb_input.last_stimulus, intensity=100)
            kb_input.last_stimulus = None
        
        # Update user presence state in hormone system
        hormone_system.set_user_presence(kb_input.user_present)
        
        # Update hormone levels
        hormone_system.update(dt)
        hormone_system.update_PB(dt)
        hormone_system.record_history()
        
        time.sleep(dt)


def main():
    print("\n" + "="*60)
    print("  REAL-TIME NEUROENDOCRINE HORMONE VISUALIZATION")
    print("  Based on: Maroto-Gómez et al. (2024)")
    print("="*60)
    print("\n📋 KEYBOARD CONTROLS:")
    print("  [H] - Hit          → increases AVP (aggression)")
    print("  [C] - Caress       → increases OT (bonding)")
    print("  [A] - Correct      → increases OT + DA (reward)")
    print("  [W] - Wrong        → increases AVP (stress)")
    print("  [U] - Toggle User Presence (one-time stimulus + baseline boost)")
    print("  [Q] - Quit")
    print("\n" + "="*60)
    print("💡 HOW USER PRESENCE WORKS:")
    print("   • Press [U] when user appears → applies stimulus ONCE")
    print("   • While present: small baseline boost to hormones")
    print("   • Press [U] again when user leaves → removes boost")
    print("   • This matches the paper: presence detected, not continuous input")
    print("="*60 + "\n")
    
    # Initialize systems
    hormone_system = HormoneSystem(PB=50)
    kb_input = KeyboardInput()
    kb_input.start()
    
    # Start simulation in background thread
    sim_thread = threading.Thread(target=simulation_loop, args=(hormone_system, kb_input), daemon=True)
    sim_thread.start()
    
    # Small delay to let thread start
    time.sleep(0.5)
    
    # Start real-time plotting (blocks until window closed)
    plotter = RealtimePlotter(hormone_system, kb_input)
    plotter.start()
    
    # Clean up
    kb_input.running = False
    print("\n✓ [SIMULATION ENDED]\n")


if __name__ == "__main__":
    main()