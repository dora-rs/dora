class BaseController:
    def __init__(self, sim_freq=60, control_freq=60):
        self.control_dt = 1 / control_freq
        self.sim_dt = 1 / sim_freq
        self.sim_time = 0

    def spawn(self, *args, **kwargs):
        pass

    def reset(self):
        self.sim_time = 0

    def forward(self, *args, **kwargs):
        self.sim_time += self.sim_dt
        if self.sim_time >= self.control_dt:
            pass
            # do something here
            self.sim_time -= self.control_dt
