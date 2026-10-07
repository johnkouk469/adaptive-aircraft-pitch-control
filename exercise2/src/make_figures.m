%% run run_pitch_control.m and save its six figures as the PNG files used in docs/report.md
run_pitch_control;

names = {'a-tracking', 'b-unstable', 'c-tracking', 'c-khat', 'c-thetahat', 'c-errors'};
outdir = fullfile(fileparts(mfilename('fullpath')), '..', 'docs', 'figures');
for k = 1:numel(names)
    print(figure(k), fullfile(outdir, [names{k} '.png']), '-dpng', '-r150', '-painters');
end
