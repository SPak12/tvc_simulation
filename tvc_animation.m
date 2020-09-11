%% TVC Animation
% Test file to play with animating model
view(gca, [0 0 1])
patchData = stlread('falcon9.stl');
p1 = patch(patchData, 'EdgeColor', 'none', 'FaceColor', '[0.8 0.8 1.0]', ...
    'FaceAlpha', 0.9);
OrigVerts = p1.Vertices;

daspect(gca, [1 1 1])
axis(gca, 'tight', 'off')
material(gca, 'dull')
camlight(gca, 'headlight')
view(gca, 3)

totrot = eye(3,3);
rot = angle2dcm(0, deg2rad(1), deg2rad(1));
for i = 1:25
    totrot = totrot*rot; % -- this applies the new rotation in the original plane axis
    p1.Vertices = (totrot*OrigVerts')';
    drawnow
end
