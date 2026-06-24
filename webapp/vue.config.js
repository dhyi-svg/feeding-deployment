const { defineConfig } = require('@vue/cli-service');
module.exports = defineConfig({
  transpileDependencies: true,
  lintOnSave: false,
  chainWebpack: config => {
    config.plugin('html').tap(args => {
      args[0].title = 'FEAST_System';
      return args;
    });
    // The project path contains square brackets (e.g. "[Winter 2026 - Present]"),
    // which copy-webpack-plugin's default absolute-path ignore for index.html
    // treats as a glob character class, so it fails to ignore public/index.html
    // and1 collides with html-webpack-plugin. Use a relative glob instead.
    config.plugin('copy').tap(args => {
      args[0].patterns[0].globOptions.ignore = ['**/.DS_Store', '**/index.html'];
      return args;
    });
  },
  devServer: {
    host: '192.168.1.2',
    allowedHosts: 'all',
    // Serve over HTTPS with a trusted cert so the iPad loads the page warning-free
    // (and can use the mic). Generate the cert/key with mkcert for 192.168.1.2.
    server: {
      type: 'https',
      options: {
        key: '/home/isacc/certs/192.168.1.2-key.pem',
        cert: '/home/isacc/certs/192.168.1.2.pem'
      }
    },
    client: {
      overlay: false,
    }
  }
});
