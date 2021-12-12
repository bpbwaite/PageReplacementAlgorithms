%% Page Replacement Algorithm Simulator
% by Bradyn Braithwaite
% 2021 Embry-Riddle Aeronautical University

clc
clearvars
close
% set the reference string and number of frames
% Example:      Reference = [7,0,1,2,0,3,0,4,2,3,0,3,2,1,2,0,1,7,0,1];
%               Frames = 3;
Reference = [7,0,1,2,0,3,0,4,2,3,0,3,2,1,2,0,1,7,0,1];
Frames = 3;

%% Example Program
a = 3;

for algorithm = 1:a
    switch(algorithm)
        case 1 % FIFO
            [framechart, faultarray, pagefaults] = pFIFO(Reference, Frames);
        case 2 % OPT
            [framechart, faultarray, pagefaults] = pOPT(Reference, Frames);
        case 3 % LRU
            [framechart, faultarray, pagefaults] = pLRU(Reference, Frames);
        otherwise
    end
    % output
    frameplot(Reference, framechart, faultarray, algorithm, a);
end

%% 'Plotting' Function
function [p] = frameplot(ref, framechart, faultarray, algo, n)
% constants
color_blk = "#000000";
color_blu = "#1111AA";
color_red = "#EE0011";
Font_Large = 16;
Methods = ["FIFO","OPTIMAL","LRU"];

xmax = length(framechart);
ymax = height(framechart) + 1;
lower_offset = 1;


persistent plotnum;
p = 0;
    if isempty(plotnum) || n == 1
        plotnum = 1;
        p = figure('Color',[1 1 1]);
    end
    subplot(n, 1, plotnum)
    plotnum = plotnum + 1;

hold on
grid off
axis([0 xmax -lower_offset ymax])
axis off
pbaspect([1 0.4 1])

% dividing lines:
for xvf = 0:xmax
    plot([xvf xvf], [0 ymax], "Color", color_blk)
end
for yvf = 0:ymax
    plot([0 xmax], [yvf yvf], "Color", color_blk)
end
plot([0 xmax], [ymax-1 ymax-1], "Color", color_blk, "LineWidth", 2)

title(['Page Replacement Algorithm: ', Methods(algo).char])
% reference string & fault-arrows:
for xit=1:xmax
    xpos = xit - 1/2;
    text(xpos, ymax - 1/2, 0, [num2str(ref(xit))],...
        "HorizontalAlignment","center","VerticalAlignment","middle",...
        "Color",color_blu,"FontName","Calibri","FontSize", Font_Large)
    
    if faultarray(xit) >= 1
        if min(framechart(:, xit)) < 0 || xit > 1 && min(framechart(:, xit-1)) < 0
            % mandatory faults
            text(xpos, lower_offset/-2, 0, ['\uparrow'],...
                "HorizontalAlignment","center","VerticalAlignment","bottom",...
                "Color",color_blk, "FontSize", Font_Large)
        else
            text(xpos, lower_offset/-2, 0, ['\uparrow'],...
                "HorizontalAlignment","center","VerticalAlignment","bottom",...
                "Color",color_red, "FontSize", Font_Large)
        end
    end
end
% full chart
for yit=1:ymax-1
    for xit=1:xmax
        if framechart(ymax - yit, xit) >= 0
            text(xit - 1/2, yit - 1/2, 0, [num2str(framechart(ymax - yit, xit))],...
                "HorizontalAlignment","center","VerticalAlignment","middle",...
                "Color",color_blk)
        end
    end
end
% subtitle
text(xmax/2, -lower_offset, 0, ['Faults: ', num2str(sum(faultarray))],...
    "HorizontalAlignment","center","VerticalAlignment","bottom")
% side titles
text(-1/2, ymax - 1/2, 0, ['Ref'],...
    "HorizontalAlignment","right","VerticalAlignment","bottom")
for alphabet=1:ymax-1
    text(-1/2, alphabet - 1/2, 0, [char(ymax-alphabet + 65 - 1)],...
        "HorizontalAlignment","right","VerticalAlignment","bottom",...
        "FontAngle","italic")
end
end

%% FIFO Algorithm
function [framechart,faultarray,pagefaults] = pFIFO(ReferenceString, Frameslots)
Numaccesses = length(ReferenceString);
framechart = zeros(Frameslots, Numaccesses) - 1;
agetracker = zeros(Frameslots, 1);
faultarray = zeros(1, Numaccesses);
for mdx=1:Numaccesses
    isprePlaced = false;
    % copy previous state
    if mdx > 1
        framechart(:, mdx) = framechart(:, mdx-1);
        agetracker = agetracker + 1;
    end
    for fdx=1:Frameslots
        % check if already in a frame
        if framechart(fdx, mdx) == ReferenceString(mdx)
            % do not reset age
            isprePlaced = true;
            break
        end
        % attempt to put in empty frame
        if framechart(fdx, mdx) < 0
            faultarray(mdx) = 1;
            isprePlaced = true;
            agetracker(fdx) = 0;
            framechart(fdx, mdx) = ReferenceString(mdx);
            break
        end
    end
    % placement failed, replace the oldest
    if ~isprePlaced
        [~, idx] = max(agetracker);
        faultarray(mdx) = 1;
        agetracker(idx) = 0;
        framechart(idx, mdx) = ReferenceString(mdx);
    end
end
pagefaults = sum(faultarray);
end
%% OPT Algorithm
function [framechart,faultarray,pagefaults] = pOPT(ReferenceString, Frameslots)
Numaccesses = length(ReferenceString);
framechart = zeros(Frameslots, Numaccesses) - 1;
futuretracker = zeros(Frameslots, 1) + Numaccesses;
faultarray = zeros(1, Numaccesses);
for mdx=1:Numaccesses
    isprePlaced = false;
    % copy previous state
    if mdx > 1
        framechart(:, mdx) = framechart(:, mdx-1);
    end
    % calculate future use distance
    if mdx < Numaccesses
        for fdx=1:Frameslots
            lookingFor = framechart(fdx, mdx);
            for ndx = mdx+1:Numaccesses
                if ReferenceString(ndx) == lookingFor
                    futuretracker(fdx) = ndx - mdx;
                    break
                else
                    % never using again:
                    futuretracker(fdx) = Numaccesses + 1;
                end
            end
        end
    end
    for fdx=1:Frameslots
        % check if already in a frame
        if framechart(fdx, mdx) == ReferenceString(mdx)
            isprePlaced = true;
            break
        end
        % attempt to put in empty frame
        if framechart(fdx, mdx) < 0
            faultarray(mdx) = 1;
            isprePlaced = true;
            framechart(fdx, mdx) = ReferenceString(mdx);
            break
        end
    end
    % placement failed, replace optimally
    if ~isprePlaced
        [~, idx] = max(futuretracker);
        faultarray(mdx) = 1;
        framechart(idx, mdx) = ReferenceString(mdx);
    end
end
pagefaults = sum(faultarray);
end
%% LRU Algorithm
function [framechart,faultarray,pagefaults] = pLRU(ReferenceString, Frameslots)
Numaccesses = length(ReferenceString);
framechart = zeros(Frameslots, Numaccesses) - 1;
agetracker = zeros(Frameslots, 1);
faultarray = zeros(1, Numaccesses);
for mdx=1:Numaccesses
    isprePlaced = false;
    % copy previous state
    if mdx > 1
        framechart(:, mdx) = framechart(:, mdx-1);
        agetracker = agetracker + 1;
    end
    for fdx=1:Frameslots
        % check if already in a frame
        if framechart(fdx, mdx) == ReferenceString(mdx)
            % in LRU, age is reset on access
            agetracker(fdx) = 0;
            isprePlaced = true;
            break
        end
        % attempt to put in empty frame
        if framechart(fdx, mdx) < 0
            faultarray(mdx) = 1;
            isprePlaced = true;
            agetracker(fdx) = 0;
            framechart(fdx, mdx) = ReferenceString(mdx);
            break
        end
    end
    % placement failed, replace the oldest,
    % which will be the least recently used.
    if ~isprePlaced
        [~, idx] = max(agetracker);
        faultarray(mdx) = 1;
        agetracker(idx) = 0;
        framechart(idx, mdx) = ReferenceString(mdx);
    end
end
pagefaults = sum(faultarray);
end
