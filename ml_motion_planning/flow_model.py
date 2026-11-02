import torch
import torch.nn as nn


class FlowMatchingModel(nn.Module):

    def __init__(self, dof=6, num_waypoints=32, max_obstacles=20):
        super().__init__()

        self.trajectory_dim = dof * num_waypoints
        self.condition_dim = 2 * dof + max_obstacles * 4

        input_dim = self.trajectory_dim + self.condition_dim + 1

        self.network = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),

            nn.Linear(512, 512),
            nn.ReLU(),

            nn.Linear(512, self.trajectory_dim)
        )

    def forward(self, x_t, t, start, goal, obstacles, obstacle_mask):

        batch_size = x_t.shape[0]

        x_t = x_t.reshape(batch_size, -1)
        obstacles = obstacles * obstacle_mask.unsqueeze(-1)
        obstacles = obstacles.reshape(batch_size, -1)

        condition = torch.cat(
            [start, goal, obstacles],
            dim=1
        )

        model_input = torch.cat(
            [x_t, t, condition],
            dim=1
        )

        return self.network(model_input)

if __name__ == "__main__":

    model = FlowMatchingModel()

    B = 8

    x_t = torch.randn(B, 192) #noise trajectory xt
    t = torch.rand(B, 1) #Flow Matching Time t

    start = torch.randn(B, 6) #Start Configuration
    goal = torch.randn(B, 6) #Goal Configuration

    obstacles = torch.randn(B, 20, 4) #Obstacles
    obstacle_mask = torch.ones(B, 20) #Obstacle Mask
    velocity = model(
        x_t, t, start, goal,
        obstacles, obstacle_mask
    )

    print("Input shape:", x_t.shape)
    print("Output shape:", velocity.shape)