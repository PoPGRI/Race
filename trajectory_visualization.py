import matplotlib.pyplot as plt
import pickle


def plot_scenario(fn_list):
    for fn in fn_list:
        res = pickle.load(open(fn, 'rb'))
        x = []
        y = []
        npc_x = []
        npc_y = []
        for i in range(len(res)):
            x.append(res[i][0])
            y.append(res[i][1])
            if len(res[i]) > 6:
                npc_x.append(res[i][6])
                npc_y.append(res[i][7])

        plt.plot(x, y)
        plt.plot(x, y, '.')
        # plt.plot(npc_x,npc_y)
    plt.show()


if __name__ == "__main__":
    fn_list = [
        "./src/race_util_module/evaluation_node/src/trajectory_ego_vehicle_Thu_Mar_25_15:30:01_2021",
        "./src/race_util_module/evaluation_node/src/trajectory_ego_vehicle_Thu_Mar_25_15:33:30_2021",
        "./src/race_util_module/evaluation_node/src/trajectory_ego_vehicle_Thu_Mar_25_15:36:34_2021"
    ]
    plot_scenario(fn_list)
