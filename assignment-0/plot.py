import pandas as pd
from matplotlib import pyplot

def plot(name: str):
    df = pd.read_csv(f'out/{name}-data.csv')

    fig, axs = pyplot.subplots(2, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(df.index, df['x'], color='blue')
    axs[0].set_title(f'Positions of {name} spring mass')
    axs[0].set_ylabel('Position')
    axs[0].grid(True)

    # Step 4: Plot y values on the second subplot
    axs[1].plot(df.index, df['y'], color='green')
    axs[1].set_title(f'Velocities of {name} spring mass')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Velocity')
    axs[1].grid(True)

    pyplot.savefig(f'out/{name}-graphs.png')

plot(name="normal")
plot(name="damper")
