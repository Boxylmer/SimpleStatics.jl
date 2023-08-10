# Example 1 - Mild Steel Warren Truss


In this example, we're going to play around with a few design parameters for a steel truss, where we want to see what size of materials we can get away with in certain sections of the truss in order to minimize the cost. 

First, we need to set up the geometry of the system. 
```@example 1
using SimpleStatics

width = (23 * 12 + 10) * 0.0254 # 23', 10" long -> Meters
height = 18 * 0.0254 # 18" tall -> Meters

n = 4

```