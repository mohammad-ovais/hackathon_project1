const path = require('path');

module.exports = {
  entry: './src/index.js', // Main entry point
  output: {
    filename: 'chat-widget.bundle.js',
    path: path.resolve(__dirname, 'dist'),
    library: 'ChatWidget',
    libraryTarget: 'umd',
    globalObject: 'this'
  },
  module: {
    rules: [
      {
        test: /\.(js|jsx)$/,
        exclude: /node_modules/,
        use: {
          loader: 'babel-loader',
          options: {
            presets: ['@babel/preset-env', '@babel/preset-react']
          }
        }
      },
      {
        test: /\.css$/,
        use: ['style-loader', 'css-loader']
      }
    ]
  },
  resolve: {
    extensions: ['.js', '.jsx']
  },
  externals: {
    'react': 'React',
    'react-dom': 'ReactDOM'
  },
  mode: 'production' // Change to 'development' for development builds
};