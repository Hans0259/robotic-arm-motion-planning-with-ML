import math

import torch
import torch.nn as nn


def straight_line_middle(start, goal, num_waypoints=32):
    """Intermediate waypoints of the straight joint-space line start -> goal.

    start, goal: (B, dof)  ->  (B, num_waypoints - 2, dof)
    """
    w = torch.linspace(0.0, 1.0, num_waypoints, device=start.device)[1:-1]
    w = w.view(1, -1, 1)
    return start.unsqueeze(1) * (1.0 - w) + goal.unsqueeze(1) * w


class ResidualBlock(nn.Module):

    def __init__(self, hidden):
        super().__init__()
        self.net = nn.Sequential(
            nn.LayerNorm(hidden),
            nn.Linear(hidden, hidden),
            nn.SiLU(),
            nn.Linear(hidden, hidden),
        )

    def forward(self, h):
        return h + self.net(h)


class FlowMatchingModel(nn.Module):
    """Velocity field v(x_t, t | start, goal, obstacles).

    x_t is the *normalized residual* of the intermediate waypoints relative to
    the straight line start -> goal, flattened to (B, dof * (num_waypoints - 2)).
    """

    def __init__(self, dof=6, num_waypoints=32, hidden=1024, emb=128, n_blocks=4):
        super().__init__()

        self.dof = dof
        self.num_waypoints = num_waypoints
        self.emb = emb
        self.trajectory_dim = dof * (num_waypoints - 2)

        # Flow time t -> sinusoidal features -> MLP
        self.time_mlp = nn.Sequential(
            nn.Linear(emb, emb),
            nn.SiLU(),
            nn.Linear(emb, emb),
        )

        # Shared per-obstacle encoder (order of obstacles does not matter)
        self.obstacle_encoder = nn.Sequential(
            nn.Linear(4, emb),
            nn.SiLU(),
            nn.Linear(emb, emb),
        )

        input_dim = self.trajectory_dim + 2 * dof + 2 * emb

        self.input_layer = nn.Linear(input_dim, hidden)
        self.blocks = nn.ModuleList(
            [ResidualBlock(hidden) for _ in range(n_blocks)]
        )
        self.output_layer = nn.Sequential(
            nn.LayerNorm(hidden),
            nn.SiLU(),
            nn.Linear(hidden, self.trajectory_dim),
        )

    def time_embedding(self, t):
        # t: (B, 1) in [0, 1]
        half = self.emb // 2
        freqs = torch.exp(
            -math.log(10000.0)
            * torch.arange(half, device=t.device, dtype=t.dtype) / half
        )
        angles = t * 1000.0 * freqs                       # (B, half)
        features = torch.cat([angles.sin(), angles.cos()], dim=1)
        return self.time_mlp(features)

    def forward(self, x_t, t, start, goal, obstacles, obstacle_mask):

        batch_size = x_t.shape[0]
        x_t = x_t.reshape(batch_size, -1)

        # Masked mean pooling over obstacles: (B, N, 4) -> (B, emb)
        mask = obstacle_mask.unsqueeze(-1)
        encoded = self.obstacle_encoder(obstacles) * mask
        obstacle_feature = encoded.sum(dim=1) / mask.sum(dim=1).clamp(min=1.0)

        model_input = torch.cat(
            [
                x_t,
                self.time_embedding(t),
                start / math.pi,          # joint angles scaled to about [-1, 1]
                goal / math.pi,
                obstacle_feature,
            ],
            dim=1,
        )

        h = self.input_layer(model_input)
        for block in self.blocks:
            h = block(h)

        return self.output_layer(h)


if __name__ == "__main__":

    model = FlowMatchingModel()

    B = 8

    x_t = torch.randn(B, model.trajectory_dim)
    t = torch.rand(B, 1)
    start = torch.randn(B, 6)
    goal = torch.randn(B, 6)
    obstacles = torch.randn(B, 20, 4)
    obstacle_mask = torch.ones(B, 20)

    velocity = model(x_t, t, start, goal, obstacles, obstacle_mask)

    print("Input shape:", x_t.shape)
    print("Output shape:", velocity.shape)
    print("Parameters:", sum(p.numel() for p in model.parameters()))
    print("Straight-line base shape:", straight_line_middle(start, goal).shape)