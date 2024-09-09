# Wait For

# Summary
Wait-For is an Agent specialized in waiting until a condition is reached on the screen. It maintains a loop looking to the screen returning when the condition has been met.

# Actions and Utils
- `wait_for`: Waits until a condition is met.

# Dependencies

- **experimental.screenshot**

# Insights

This agent simply waits in an infinite loop asking itself if the condition has been reached, if not, sleeps and iterates again.
