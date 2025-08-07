javaaddpath(init.build_path('../../lib/jar/postgresql-42.7.7.jar'));

DB_CONFIG.host = 'localhost';
DB_CONFIG.port = 5432;
DB_CONFIG.name = 'drone';
DB_CONFIG.user = 'postgres';
DB_CONFIG.pass = '';

try

    conn = database(DB_CONFIG.name, DB_CONFIG.user, DB_CONFIG.pass, ...
        'org.postgresql.Driver', ['jdbc:postgresql://' DB_CONFIG.host ':' num2str(DB_CONFIG.port) '/' DB_CONFIG.name]);

    if isempty(conn.Message)
        fprintf('数据库连接成功！\n\n');
        conn.AutoCommit = 'off'; % 开启事务模式
    else
        error(conn.Message);
    end

catch ME
    error('数据库连接失败: %s', ME.message);
end
