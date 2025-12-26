/**
 * Docusaurus plugin for RAG Chatbot integration
 * This plugin adds the chatbot widget to all documentation pages
 */

const path = require('path');

module.exports = function (context, options) {
  const { siteConfig } = context;
  const config = {
    backendUrl: options.backendUrl || '',
    apiKey: options.apiKey || '',
    theme: options.theme || 'light',
    position: options.position || 'right',
    ...options,
  };

  return {
    name: 'docusaurus-plugin-rag-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './chatbot-client')];
    },

    injectHtmlTags() {
      return {
        postBodyTags: [
          `<script>
            // Initialize the chatbot widget after page loads
            document.addEventListener('DOMContentLoaded', function() {
              if (typeof window.ChatWidget !== 'undefined') {
                window.ChatWidget.init({
                  backendUrl: '${config.backendUrl}',
                  apiKey: '${config.apiKey}',
                  theme: '${config.theme}',
                  position: '${config.position}'
                });
              } else {
                console.warn('ChatWidget not found. Make sure the widget script is loaded.');
              }
            });
          </script>`,
        ],
      };
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@chatbot-plugin': path.resolve(__dirname, '.'),
          },
        },
      };
    },
  };
};