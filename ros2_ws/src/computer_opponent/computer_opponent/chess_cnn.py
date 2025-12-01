from torch import nn, optim

class ChessCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 12, 3, padding=1),  # input: 3 channels
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(12, 24, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(24, 24, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Dropout(0.4),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(24 * 8 * 8, 3),  # 96/8 = 12 after 3 pooling layers
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x