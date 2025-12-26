/**
 * Embeddable JavaScript widget for the RAG Chatbot
 * This script can be included on any page to add the chat functionality
 */

(function() {
  'use strict';

  // Configuration object
  let config = {
    backendUrl: '',
    apiKey: '',
    theme: 'light',  // 'light' or 'dark'
    position: 'right'  // 'left' or 'right'
  };

  // Initialize the widget
  function init(options = {}) {
    // Merge provided options with defaults
    config = { ...config, ...options };

    // Validate required options
    if (!config.backendUrl) {
      console.error('ChatWidget: backendUrl is required');
      return;
    }

    // Create the chat widget container
    createWidget();

    // Load React and render the widget
    loadReactAndRender();
  }

  // Create the widget container
  function createWidget() {
    // Check if widget already exists
    if (document.getElementById('chat-widget-container')) {
      return;
    }

    // Create a container for our React app
    const container = document.createElement('div');
    container.id = 'chat-widget-container';
    container.style.display = 'none'; // Initially hidden
    document.body.appendChild(container);
  }

  // Load React and render the widget
  async function loadReactAndRender() {
    // Check if React is already loaded
    if (window.React && window.ReactDOM) {
      renderWidget();
      return;
    }

    // Load React and ReactDOM from CDN
    const reactScript = document.createElement('script');
    reactScript.src = 'https://unpkg.com/react@18/umd/react.production.min.js';
    document.head.appendChild(reactScript);

    const reactDOMScript = document.createElement('script');
    reactDOMScript.src = 'https://unpkg.com/react-dom@18/umd/react-dom.production.min.js';
    reactDOMScript.onload = () => {
      // Wait a bit for React to be fully loaded
      setTimeout(renderWidget, 100);
    };
    document.head.appendChild(reactDOMScript);
  }

  // Render the widget using React
  function renderWidget() {
    if (!window.React || !window.ReactDOM) {
      console.error('ChatWidget: Failed to load React');
      return;
    }

    const React = window.React;
    const ReactDOM = window.ReactDOM;

    // Define the ChatWidget component
    const ChatWidgetComponent = () => {
      const [isOpen, setIsOpen] = React.useState(false);
      const [messages, setMessages] = React.useState([]);
      const [inputValue, setInputValue] = React.useState('');
      const [isLoading, setIsLoading] = React.useState(false);
      const [sessionId, setSessionId] = React.useState(null);

      // Initialize session on component mount
      React.useEffect(() => {
        // Generate a session ID if one doesn't exist
        if (!sessionId) {
          const id = localStorage.getItem('chatSessionId') || `session_${Date.now()}`;
          localStorage.setItem('chatSessionId', id);
          setSessionId(id);
        }
      }, [sessionId]);

      const toggleChat = () => {
        setIsOpen(!isOpen);
      };

      const sendMessage = async () => {
        if (!inputValue.trim() || isLoading) return;

        const userMessage = { role: 'user', content: inputValue, timestamp: new Date() };
        setMessages(prev => [...prev, userMessage]);
        setInputValue('');
        setIsLoading(true);

        try {
          const response = await fetch(`${config.backendUrl}/api/chat/query`, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              query: inputValue,
              session_id: sessionId,
              include_citations: true
            })
          });

          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }

          const data = await response.json();

          const botMessage = {
            role: 'assistant',
            content: data.answer,
            citations: data.citations,
            timestamp: new Date()
          };

          setMessages(prev => [...prev, botMessage]);
        } catch (error) {
          console.error('Error sending message:', error);
          const errorMessage = {
            role: 'assistant',
            content: 'Sorry, I encountered an error. Please try again.',
            timestamp: new Date()
          };
          setMessages(prev => [...prev, errorMessage]);
        } finally {
          setIsLoading(false);
        }
      };

      const handleKeyPress = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
          e.preventDefault();
          sendMessage();
        }
      };

      // Apply theme and position classes
      const widgetClasses = [
        'chat-widget',
        `chat-theme-${config.theme}`,
        `chat-position-${config.position}`
      ].join(' ');

      return React.createElement(
        'div',
        { className: widgetClasses },
        isOpen
          ? React.createElement(
              'div',
              { className: 'chat-window' },
              React.createElement(
                'div',
                { className: 'chat-header' },
                React.createElement('h3', null, 'Textbook Assistant'),
                React.createElement(
                  'button',
                  { className: 'close-btn', onClick: toggleChat },
                  '×'
                )
              ),
              React.createElement(
                'div',
                { className: 'chat-messages' },
                messages.map((msg, index) =>
                  React.createElement(
                    'div',
                    { key: index, className: `message ${msg.role}` },
                    React.createElement(
                      'div',
                      { className: 'message-content' },
                      msg.content,
                      msg.citations && msg.citations.length > 0 && React.createElement(
                        'div',
                        { className: 'citations' },
                        React.createElement('strong', null, 'Citations:'),
                        React.createElement(
                          'ul',
                          null,
                          msg.citations.map((citation, idx) =>
                            React.createElement(
                              'li',
                              { key: idx },
                              React.createElement(
                                'a',
                                { href: citation.page_url, target: '_blank', rel: 'noopener noreferrer' },
                                `${citation.chapter} - ${citation.section}`
                              )
                            )
                          )
                        )
                      )
                    )
                  )
                ),
                isLoading && React.createElement(
                  'div',
                  { className: 'message assistant' },
                  React.createElement('div', { className: 'message-content' }, 'Thinking...')
                )
              ),
              React.createElement(
                'div',
                { className: 'chat-input-area' },
                React.createElement('textarea', {
                  value: inputValue,
                  onChange: (e) => setInputValue(e.target.value),
                  onKeyPress: handleKeyPress,
                  placeholder: 'Ask about the textbook content...',
                  disabled: isLoading
                }),
                React.createElement(
                  'button',
                  { onClick: sendMessage, disabled: isLoading || !inputValue.trim() },
                  'Send'
                )
              )
            )
          : React.createElement(
              'button',
              { className: 'chat-toggle-btn', onClick: toggleChat },
              '💬 Ask Textbook'
            )
      );
    };

    // Render the component
    const container = document.getElementById('chat-widget-container');
    ReactDOM.render(React.createElement(ChatWidgetComponent), container);
    container.style.display = 'block';
  }

  // Add default styles
  function addDefaultStyles() {
    if (document.getElementById('chat-widget-styles')) {
      return;
    }

    const style = document.createElement('style');
    style.id = 'chat-widget-styles';
    style.textContent = `
      .chat-widget {
        position: fixed;
        bottom: 20px;
        z-index: 10000;
      }

      .chat-widget.chat-position-right {
        right: 20px;
      }

      .chat-widget.chat-position-left {
        left: 20px;
      }

      .chat-toggle-btn {
        background-color: #4f46e5;
        color: white;
        border: none;
        border-radius: 50%;
        width: 60px;
        height: 60px;
        font-size: 24px;
        cursor: pointer;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .chat-toggle-btn:hover {
        background-color: #4338ca;
      }

      .chat-window {
        width: 350px;
        height: 500px;
        border: 1px solid #e5e7eb;
        border-radius: 8px;
        box-shadow: 0 10px 15px rgba(0, 0, 0, 0.1);
        display: flex;
        flex-direction: column;
        background-color: white;
      }

      .chat-header {
        background-color: #4f46e5;
        color: white;
        padding: 12px;
        border-top-left-radius: 8px;
        border-top-right-radius: 8px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }

      .chat-header h3 {
        margin: 0;
        font-size: 16px;
      }

      .close-btn {
        background: none;
        border: none;
        color: white;
        font-size: 20px;
        cursor: pointer;
        padding: 0;
        width: 24px;
        height: 24px;
        display: flex;
        align-items: center;
        justify-content: center;
      }

      .chat-messages {
        flex: 1;
        padding: 12px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 12px;
      }

      .message {
        max-width: 80%;
      }

      .message.user {
        align-self: flex-end;
      }

      .message.assistant {
        align-self: flex-start;
      }

      .message-content {
        padding: 8px 12px;
        border-radius: 18px;
        word-wrap: break-word;
      }

      .message.user .message-content {
        background-color: #4f46e5;
        color: white;
      }

      .message.assistant .message-content {
        background-color: #f3f4f6;
        color: #374151;
      }

      .citations {
        margin-top: 8px;
        padding-top: 8px;
        border-top: 1px solid #e5e7eb;
        font-size: 12px;
      }

      .citations ul {
        margin: 4px 0;
        padding-left: 16px;
      }

      .citations li {
        margin: 2px 0;
      }

      .citations a {
        color: #4f46e5;
        text-decoration: none;
      }

      .citations a:hover {
        text-decoration: underline;
      }

      .chat-input-area {
        padding: 12px;
        border-top: 1px solid #e5e7eb;
        display: flex;
        gap: 8px;
      }

      .chat-input-area textarea {
        flex: 1;
        padding: 8px;
        border: 1px solid #d1d5db;
        border-radius: 4px;
        resize: none;
        min-height: 40px;
        max-height: 100px;
      }

      .chat-input-area button {
        padding: 8px 16px;
        background-color: #4f46e5;
        color: white;
        border: none;
        border-radius: 4px;
        cursor: pointer;
      }

      .chat-input-area button:disabled {
        background-color: #9ca3af;
        cursor: not-allowed;
      }

      /* Dark theme */
      .chat-theme-dark .chat-window {
        background-color: #1f2937;
        color: #f9fafb;
        border-color: #374151;
      }

      .chat-theme-dark .chat-header {
        background-color: #374151;
      }

      .chat-theme-dark .chat-messages {
        background-color: #1f2937;
      }

      .chat-theme-dark .message.assistant .message-content {
        background-color: #374151;
        color: #d1d5db;
      }

      .chat-theme-dark .chat-input-area {
        border-top-color: #374151;
      }

      .chat-theme-dark .chat-input-area textarea {
        background-color: #374151;
        color: #f9fafb;
        border-color: #4b5563;
      }
    `;
    document.head.appendChild(style);
  }

  // Add styles when the script loads
  addDefaultStyles();

  // Expose the API globally
  window.ChatWidget = {
    init: init,
    open: () => {
      // Implementation for opening the chat
      const event = new CustomEvent('chatWidgetOpen');
      document.dispatchEvent(event);
    },
    close: () => {
      // Implementation for closing the chat
      const event = new CustomEvent('chatWidgetClose');
      document.dispatchEvent(event);
    }
  };

  // Auto-initialize if configuration is provided via data attributes
  document.addEventListener('DOMContentLoaded', () => {
    const widgetScript = document.currentScript || document.querySelector('script[src*="chat-widget"]');
    if (widgetScript) {
      const backendUrl = widgetScript.getAttribute('data-backend-url');
      const apiKey = widgetScript.getAttribute('data-api-key');
      const theme = widgetScript.getAttribute('data-theme') || 'light';
      const position = widgetScript.getAttribute('data-position') || 'right';

      if (backendUrl) {
        init({ backendUrl, apiKey, theme, position });
      }
    }
  });
})();