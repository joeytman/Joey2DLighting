# Joey2DLighting
This project is an attempt to build a 2D Dynamic Lighting solution for Unity with cleaner code and documentation along with better optimization than the current most popular solution, 2DDL by Martin Ysa. This project is a WIP!

# Functionality
Here's an example of the lighting rendering as it stands now:

![Dragging around light](https://github.com/joeytman/Joey2DLighting/blob/master/Images/Showcase_Brief.gif?raw=true)

This gif showcases a few of the different raytracing methods that are used in conjunction to create the lighting mesh:

![Showcasing options](https://github.com/joeytman/Joey2DLighting/blob/master/Images/Demonstrating_Options.gif?raw=true)

# Goals
Key to this project is implementing lighting such that restricted angle lighting comes with an actual performance increase, lessening the performance hit for games and projects that rely on many dynamic lights that only cast lighting in a small arc (e.g. streetlights, flashlights, etc). This is a core optimization that 2DDL lacks, and the goal is to have the performance of each angled light scale linearly (or at least close to that) with the angle of light being casted (O(n) where 0 <= n <= 360, the degree of the angle of light cast).

# Inspiration
This project was inspired by a game I was working on last year with a small team that relied on heavy use of dynamic 2d lighting. We chose to use 2DDL for our project as it was the most popular, but after hitting a wall with trying to bugfix some strange issues, I read through the source code of 2DDL and noticed glaring issues with the creator's method of approximating trigonometric functions. 

While he accounted for those errors within the lighting script through an abundance of conditional statements to account for the fundamentally incorrect trigonometric approximations, there were enough unchecked errors to result in our game's lighting breaking in situations that made the game unacceptable to ship. Hoping for an easy fix, I decided to rewrite the script that approximated the trigonometric functions. 

I noticed that the precise values of sine and cosine weren't very important in an absolute way, only relative to all the other vertices that the raycasts hit, and his approximation function didn't preserve the periodic wave-like characteristics of those functions, which lead to such major issues. 

Rewriting those functions to fix them resulted in his lighting implementation completely breaking, and with all the documentation being a random mix of comments in English and Spanish coupled with unreadably long chunks of undocumented code, I decided to just start from scratch by learning about Unity raycasting and dynamic mesh generation and begin working on my own. Thus, this passion project was born. 

# Progress
As of 3/17/18, there are a few bugs with the unrestricted angled lighting but it mostly works well. The angled lighting works well when restricted to certain angles but breaks on others, and I've mostly narrowed down why this is happening, but my courseload has prevented me from being able to work on it at all in the last few months. Hopefully I'll soon be able to sit down and hammer those out, as if I can, I'd be ready to publish the asset on the Unity store promising barebones but customizable and well-optimized 2D dynamic lighting for free. After I reach that goal, I plan to continue adding features to bring my solution towards feature parity with 2DDL during breaks and times away from school. 
