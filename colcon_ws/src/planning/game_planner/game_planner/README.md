# game_planner package

## Description

This package contains the main state machine of the project, which is responsible for the general behavior of the robot during the game..

## Workflow

```mermaid
graph TD
    A[Game Controller Starts] --> B[INITIAL_STATE]
    B --> C[STATE_READY]
    C --> D[STATE_SET]
    D --> E[STATE_PLAYING]
    E --> F[STATE_FINISHED]
```
> ℹ️ **to-do**: Finish the documentation.