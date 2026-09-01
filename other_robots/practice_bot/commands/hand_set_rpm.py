import commands2
from helpers.log_command import log_command  # outsource explicit logging clutter to a single line

from subsystems.hand import Hand
from subsystems.led import Led
import math

@log_command(console=True, nt=False, print_init=True, print_end=False)  # will print start and end messages
class Hand_Set_RPM(commands2.Command):  # change the name for your command


    def __init__(self, hand: Hand, rpm=1000, on_start=False, indent=0, led:Led=None) -> None:
        super().__init__()
        self.setName('Hand_Set')
        self.rpm = rpm
        self.hand = hand
        self.indent = indent
        self.on_start = on_start
        self.led = led
        self.previous_rpm = 0
        self.addRequirements(self.hand)

    def initialize(self) -> None:
        self.previous_rpm = self.hand.get_rpm()
        #self.hand.set_hand_rpm(self.rpm) if math.fabs(self.rpm) > 1 else self.hand.stop_hand()

        self.extra_log_info = f"RPM={self.rpm}"

        if self.hand.current_rpm > 10:
            self.hand.stop_hand()
        else:
            self.hand.set_hand_rpm(self.rpm)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True
        
    def end(self, interrupted: bool) -> None:
        pass

        """
        if self.led is not None:
            if abs(self.rpm) > 1:
                self.led.set_indicator(Led.Indicator.kINTAKEON)
            elif abs(self.previous_rpm) > 1:
                commands2.CommandScheduler.getInstance().schedule(
                    self.led.set_indicator_with_timeout(Led.Indicator.kINTAKEOFF, timeout=1.5))
            else:
                pass
        """