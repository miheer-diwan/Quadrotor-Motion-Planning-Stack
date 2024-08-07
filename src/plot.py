import scipy.io
import matplotlib.pyplot as plt

class UserStatesPlotter:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = self.load_user_states_data()
        self.x_data = self.data['x']
        self.y_data = self.data['y']
        self.z_data = self.data['z']
        self.vx_data = self.data['vx']
        self.vy_data = self.data['vy']
        self.vz_data = self.data['vz']

        self.x_des_data = self.data['x_des']
        self.y_des_data = self.data['y_des']
        self.z_des_data = self.data['z_des']
        self.vx_des_data = self.data['vx_des']
        self.vy_des_data = self.data['vy_des']
        self.vz_des_data = self.data['vz_des']

        # print(self.x_data)

        self.time_data = self.data['time']

    def load_user_states_data(self):
        """
        Load data from 'user_states.mat' file.
        Returns:
            dict: Loaded data from the file.
        """
        return scipy.io.loadmat(self.file_path)

    def plot(self):
        """
        Create a plot to visualize 'x', 'y', 'z' positions over time.
        """
        plt.figure(figsize=(12, 6))

        plt.subplot(2, 3, 1)
        plt.plot(self.time_data, self.x_data, label='x')
        plt.plot(self.time_data, self.x_des_data, label='x_des')
        plt.xlabel('Time')
        plt.ylabel('X Position')
        plt.legend()
        plt.title('Position x vs. Time')

        plt.subplot(2, 3, 2)
        plt.plot(self.time_data, self.y_data, label='y')
        plt.plot(self.time_data, self.y_des_data, label='y_des')
        plt.xlabel('Time')
        plt.ylabel('Y Position')
        plt.legend()
        plt.title('Position y vs. Time')

        plt.subplot(2, 3, 3)
        plt.plot(self.time_data, self.z_data, label='z')
        plt.plot(self.time_data, self.z_des_data, label='z_des')
        plt.xlabel('Time')
        plt.ylabel('Z Position')
        plt.legend()
        plt.title('Position z vs. Time')

        plt.subplot(2, 3, 4)
        plt.plot(self.time_data, self.vx_data, label='vx')
        plt.plot(self.time_data, self.vx_des_data, label='vx_des')
        plt.xlabel('Time')
        plt.ylabel('X Velocity')
        plt.legend()
        plt.title('Velocity x vs. Time')

        plt.subplot(2, 3, 5)
        plt.plot(self.time_data, self.vy_data, label='vy')
        plt.plot(self.time_data, self.vy_des_data, label='vy_des')
        plt.xlabel('Time')
        plt.ylabel('Y Velocity')
        plt.legend()
        plt.title('Velocity y vs. Time')

        plt.subplot(2, 3, 6)
        plt.plot(self.time_data, self.vz_data, label='vz')
        plt.plot(self.time_data, self.vz_des_data, label='vz_des')
        plt.xlabel('Time')
        plt.ylabel('Z Velocity')
        plt.legend()
        plt.title('Velocity z vs. Time')

        plt.tight_layout()
        plt.show()

    def plot_trajectory_3d(self):
        """
        Create a 3D plot to visualize 'x', 'y', 'z', 'x_des', 'y_des', 'z_des' positions over time.
        """
        plt.figure(figsize=(10, 8))
        ax = plt.axes(projection='3d')

        ax.plot(self.x_data.flatten(), self.y_data.flatten(), self.z_data.flatten(), label='Actual Trajectory')
        ax.plot(self.x_des_data.flatten(), self.y_des_data.flatten(), self.z_des_data.flatten(), label='Desired Trajectory', linestyle='--')

        # ax.set_xlabel('X Position')
        # ax.set_ylabel('Y Position')
        # ax.set_zlabel('Z Position')
        ax.legend()
        ax.set_title('3D Trajectory')

        plt.show()

def main():
    file_path = '../log/user_states.mat'
    plotter = UserStatesPlotter(file_path)
    plotter.plot()
    plotter.plot_trajectory_3d()

if __name__ == "__main__":
    main()
