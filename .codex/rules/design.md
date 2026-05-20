# Design Rules

## App UI

- AMR2.0 is an operational robotics dashboard, so prioritize dense, scannable,
  repeatable workflows.
- Avoid landing-page style layouts for tools. The first screen should be usable.
- Keep controls familiar: icons for tool buttons, toggles for binary options,
  sliders/inputs for numeric tuning, tabs for views.
- For RViz and map tools, preserve spatial consistency across pose, map,
  costmap, path, and robot heading.
- Avoid layout shifts: fixed-format controls need stable dimensions.

## Visual Checks

- For UI changes, verify with a browser screenshot when practical.
- Make sure text does not overlap or overflow on compact panels.
- For robot heading or map rotation changes, visual confirmation matters in
  addition to tests.
