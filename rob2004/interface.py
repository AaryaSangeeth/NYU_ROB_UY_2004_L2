import zerorpc
import numpy as np

class RobotInterfaceSim:
    def __init__(self, ip='127.0.0.1', port=4242):
        self.robot = zerorpc.Client()
        # Connect to the server's address
        self.robot.connect(f"tcp://{ip}:{port}")

    def getJointState(self):
        state = self.robot.getJointState()
        if state is not None:
            return {k:np.array(v) for k,v in state.items()}
        else:
            return None
        
    def setJointCommand(self, tau):
        if isinstance(tau, np.ndarray):
            assert tau.shape==(12,), 'The shape of the torque vector tau should be (12,)!'
            tau_ = tau.tolist()
        elif isinstance(tau, list):
            assert len(tau)==12, 'The length of the tau list should be 12!'
            tau_ = tau
        else:
            raise TypeError('Tau can only be a numpy array or python list with length 12')
        
        self.robot.setJointCommand(tau_)
