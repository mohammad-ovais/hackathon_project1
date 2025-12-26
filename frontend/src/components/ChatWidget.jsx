import React, { useState, useEffect } from 'react';
import './ChatWidget.css';

const ChatWidget = ({ backendUrl }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);

  // Initialize session on component mount
  useEffect(() => {
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
      const response = await fetch(`${backendUrl}/api/chat/query`, {
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

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>Textbook Assistant</h3>
            <button className="close-btn" onClick={toggleChat}>×</button>
          </div>
          <div className="chat-messages">
            {messages.map((msg, index) => (
              <div key={index} className={`message ${msg.role}`}>
                <div className="message-content">
                  {msg.content}
                  {msg.citations && msg.citations.length > 0 && (
                    <div className="citations">
                      <strong>Citations:</strong>
                      <ul>
                        {msg.citations.map((citation, idx) => (
                          <li key={idx}>
                            <a href={citation.page_url} target="_blank" rel="noopener noreferrer">
                              {citation.chapter} - {citation.section}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">Thinking...</div>
              </div>
            )}
          </div>
          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the textbook content..."
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !inputValue.trim()}>
              Send
            </button>
          </div>
        </div>
      ) : (
        <button className="chat-toggle-btn" onClick={toggleChat}>
          💬 Ask Textbook
        </button>
      )}
    </div>
  );
};

export default ChatWidget;