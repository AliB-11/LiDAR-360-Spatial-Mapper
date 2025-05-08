fig = openfig('new_color_scan.fig');  % open the figure
all_scatters = findall(fig, 'Type', 'Scatter');  % scatter3 plots are of this type

for k = 1:length(all_scatters)
    all_scatters(k).Visible = 'off';  % simply hides them
end

all_lines = findall(fig, 'Type', 'Line');

% Reverse the order so earliest-drawn lines get earliest colors
all_lines = flipud(all_lines);

% Choose a smooth colormap
cmap = turbo(length(all_lines));  % try parula, hot, or copper too

% Assign a gradient color
for k = 1:length(all_lines)
    all_lines(k).Color = cmap(k,:);
end