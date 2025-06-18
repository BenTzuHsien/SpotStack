import time
from bosdyn.api import robot_state_pb2
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.power import PowerClient, power_on, safe_power_off

class PowerManager:
    """
    Manages Spot's motor power state, including power-on and safe shutdown.
    """

    def __init__(self, robot):
        """
        Initialize the PowerManager class.

        Parameters
        ----------
        robot : bosdyn.client.robot.Robot
            The Spot robot instance.
        """
        # Create robot clients
        self._state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)

    def check_is_powered_on(self):
        """
        Check whether the robot is currently powered on.

        Returns
        -------
        bool
            True if Spot's motors are on, False otherwise.
        """
        power_state = self._state_client.get_robot_state().power_state
        powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        return powered_on
    
    def toggle_power(self, should_power_on):
        """
        Power the robot on/off dependent on the current power state.
        
        Parameters
        ----------
        should_power_on : bool
            If True, attempts to power on. If False, powers off if needed.

        Returns
        -------
        bool
            True if the robot is powered on after the operation, False otherwise.
        """
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._state_client.get_robot_state_async()
                state_response = future.result(timeout=10)  # 10 second timeout for waiting for the state response.
                if state_response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
                    motors_on = True
                else:
                    # Motors are not yet fully powered on.
                    time.sleep(.25)
        elif is_powered_on and not should_power_on:
            # Safe power off (robot will sit then power down) when it is in a powered-on state.
            safe_power_off(self._command_client, self._state_client)
        else:
            # Return the current power state without change.
            return is_powered_on
        # Update the locally stored power state.
        is_powered_on = self.check_is_powered_on()
        return is_powered_on
    
# Example Usage
if __name__ == '__main__':

    import argparse, bosdyn.client.util, sys
    from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
    from bosdyn.client.robot_command import blocking_stand
    
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(sys.argv[1:])

    # Create robot object
    sdk = bosdyn.client.create_standard_sdk('PowerManager')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    try:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            try:
                power_manager = PowerManager(robot)

                is_power_on = power_manager.check_is_powered_on()
                print(f'If the robot is power on: {is_power_on}')

                power_manager.toggle_power(should_power_on=True)
                is_power_on = power_manager.check_is_powered_on()
                print(f'If the robot is power on: {is_power_on}')

                time.sleep(2)
                blocking_stand(command_client)
                time.sleep(2)

                power_manager.toggle_power(should_power_on=False)
                is_power_on = power_manager.check_is_powered_on()
                print(f'If the robot is power on: {is_power_on}')

            except Exception as exc:  # pylint: disable=broad-except
                print("PowerManager threw an error.")
                print(exc)

    except ResourceAlreadyClaimedError:
        print(
            "The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds."
        )