function Smooth_path = Routing_Smoothing(Path)
    Path_x = Path(1,:);
    Path_y = Path(2,:);
    Smooth_path = spcrv([[Path_x(1) Path_x Path_x(end)];[Path_y(1) Path_y Path_y(end)]],3);
end