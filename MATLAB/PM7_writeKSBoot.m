%PM7_writeBootFile.m
%Writes a .ks boot file, for automatic interoperability between MATLAB and kOS.
%Will keep track of written files, eliminating the risk of overwrite.
if (exist('wk_count','var'))==0
    wk_count = 0;
end
wk_fn = vehicleName;
%check for proper program size
[wk_n1,wk_n2] = size(s1_prog);
if wk_n1<3 || wk_n2 ~= 2
    return;
end;
%open file for write
wk_count = wk_count+1;
wk_fn = strcat('boot_',strrep(wk_fn,' ','_'),sprintf('%d',wk_count),'.ks');
wk_f = fopen(wk_fn, 'w');

%write vehicle name
fprintf(wk_f, 'GLOBAL P_vName IS "%s".\n\n', vehicleName);

%write launch sequence (completed by user)
fprintf(wk_f, 'GLOBAL P_seq IS LIST(%4.1f, , , %4.1f, , , , , %4.1f).//remember to fill this!\n', -s1_engT, s1_maxT-s1_engT, s2_maxT);
fprintf(wk_f, '//ignition (delay before release), booster jettison, fairings jettison, cutoff, separation, ullage, ignition, PEG activation, stage 2 maxT\n\n');

%write time table
fprintf(wk_f, 'GLOBAL P_pt IS LIST(');
for i = 1:wk_n1-1
    fprintf(wk_f, '%8.4f, ', s1_prog(i,1));
end;
fprintf(wk_f, '%8.4f).\n', s1_prog(wk_n1,1));
%write pitch table
fprintf(wk_f, 'GLOBAL P_pp IS LIST(');
for i = 1:wk_n1-1
    fprintf(wk_f, '%8.4f, ', s1_prog(i,2));
end;
fprintf(wk_f, '%8.4f).\n\n', s1_prog(wk_n1,2));

%write ullage setting line (completed by user)
fprintf(wk_f, 'GLOBAL P_umode IS .//remember to fill this!\n\n');

%write terminal activation line
fprintf(wk_f, 'TOGGLE AG10.\n\n');

%final and most important
fprintf(wk_f, 'RUN pegas_loader.ks.');

%close file
fclose(wk_f);