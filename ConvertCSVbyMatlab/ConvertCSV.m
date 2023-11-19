% A列 Time, B列 θのCSVを読み込み、
% Arduinoスケッチで使えるヘッダーファイルに変換するコード。

% θ1.csvの読み込み
data1 = readtable('θ1.csv', 'VariableNamingRule', 'preserve');
linkValues = data1.("ƒÆ1");

% θ2.csvの読み込み
data2 = readtable('θ2.csv', 'VariableNamingRule', 'preserve');
beltValues = data2.("ƒ¦2[“x]");

% ヘッダーファイルの内容の生成
headerContent = "const float LinktgtAngle[" + length(linkValues) + "] = {";
for i = 1:length(linkValues)
    headerContent = headerContent + linkValues(i) + ", ";
    if mod(i, 10) == 0
        headerContent = headerContent + newline;
    end
end
headerContent = headerContent + "};" + newline + newline;

headerContent = headerContent + "const float BelttgtAngle[" + length(beltValues) + "] = {";
for i = 1:length(beltValues)
    headerContent = headerContent + beltValues(i) + ", ";
    if mod(i, 10) == 0
        headerContent = headerContent + newline;
    end
end
headerContent = headerContent + "};";

% ヘッダーファイルの書き出し
fid = fopen('DualMotorPID_Trajectory.h', 'wt');
fprintf(fid, headerContent);
fclose(fid);
