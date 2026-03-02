# STIMULUS TABLE (Eq. 3 mapping)
class StimulusTable:
    def __init__(self):
        self.table = {
            "HIT": [
                {"target": "AVP", "alpha": 0.0075, "beta": 1.0}
            ],
            "CARESS": [
                {"target": "OT", "alpha": 0.0075, "beta": "0.02 * PB"},
                {"target": "SN", "alpha": -0.002, "beta": 1.0}  # Physiological inhibition
            ],
            "CORRECT": [
                {"target": "OT", "alpha": 0.001, "beta": "0.02 * PB"},
                {"target": "DA", "alpha": 0.005, "beta": 1.0},
            ],
            "WRONG": [
                {"target": "AVP", "alpha": 0.002, "beta": 1.0}
            ],
            "USER_PRESENCE": [
                {"target": "AVP", "alpha": 0.003, "beta": 1.0},
                {"target": "OT",  "alpha": 0.003, "beta": "0.02 * PB"},
                {"target": "DA",  "alpha": 0.004, "beta": 1.0},
            ],
        }

    def get_effects(self, stim):
        return self.table.get(stim, [])

