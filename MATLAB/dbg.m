figure(3); clf;
N = p_stage2.Plots.DEBUG.THIS;
K = 173;
t = p_stage2.Plots.DEBUG.time(6:N);

a = p_stage2.Plots.DEBUG.iF_plane(6:N,:);
b = p_stage2.Plots.DEBUG.iF_up(6:N,:);

show = 2;   %how many items to show? 1 or 2
dims = 1;   %1 shows linear plot (of norm, if passed a 4D item), 3 shows 3D plot

s = size(a);
if dims==3 && s(2)<3
    dims = 1;
end;
if dims==3
    axis vis3d;
    hold on;
    plot3(a(:,1), a(:,2), a(:,3), 'b');
    scatter3(a(1,1), a(1,2), a(1,3), 'r');
    scatter3(a(K,1), a(K,2), a(K,3), 'k');
    if show>1
        plot3(b(:,1), b(:,2), b(:,3), 'g');
        scatter3(b(1,1), b(1,2), b(1,3), 'r');
        scatter3(b(K,1), b(K,2), b(K,3), 'k');
    end;
    hold off;
else
    hold on;
    if s(2)>3
        dim=4;
    else
        dim=1;
    end;
    plot(t, a(:,dim), 'b');
    scatter(t(K), a(K,dim), 'k');
    if show>1
        plot(t, b(:,dim), 'g');
        scatter(t(K), b(K,dim), 'k');
    end;
    hold off;
end;

clearvars N K t a b show dims s dim