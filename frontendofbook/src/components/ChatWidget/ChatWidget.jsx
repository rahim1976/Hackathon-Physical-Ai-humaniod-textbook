import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ChatWidget.module.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toLocaleTimeString()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    const currentInputValue = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request - include selected text if available
      let messageToSend = currentInputValue;
      if (selectedText) {
        messageToSend = `Context: ${selectedText}\n\nQuestion: ${currentInputValue}`;
      }

      const response = await fetch('https://rahimalidev-ai-hackathon.hf.space/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: currentInputValue,
          selected_text: selectedText,
          context: selectedText ? `Selected text: ${selectedText}` : ''
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const agentMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'agent',
        sources: data.sources || [],
        timestamp: new Date().toLocaleTimeString()
      };

      setMessages(prev => [...prev, agentMessage]);

      // Clear selected text after successful query
      setSelectedText('');
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'agent',
        timestamp: new Date().toLocaleTimeString()
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
    <>
      {/* Floating Chat Button */}
      <button
        className={clsx(styles.floatingBtn, {[styles.hidden]: isOpen})}
        onClick={toggleChat}
        aria-label="Open chat"
      >
        <i className={clsx('fas', 'fa-comment', styles.chatIcon)}></i>
      </button>

      {/* Chat Widget Container */}
      <div className={clsx(styles.chatWidget, {[styles.open]: isOpen})}>
        <div className={styles.chatHeader}>
          <h3>AI Assistant</h3>
          <button
            className={styles.closeBtn}
            onClick={toggleChat}
            aria-label="Close chat"
          >
            <i className="fas fa-times"></i>
          </button>
        </div>


        <div className={styles.chatMessages}>
          {messages.length === 0 ? (
            <div className={styles.welcomeMessage}>
              Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about robotics, ROS2, humanoid design, or related topics.
              {selectedText && (
                <div className={styles.selectedTextIndicator}>
                  Selected text context available
                </div>
              )}
            </div>
          ) : (
            messages.map((message) => (
              <div
                key={message.id}
                className={clsx(
                  styles.message,
                  styles[`${message.sender}Message`]
                )}
              >
                <div className={styles.messageText}>{message.text}</div>
                <div className={styles.messageTime}>{message.timestamp}</div>

                {message.sender === 'agent' && message.sources && message.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    {message.sources.slice(0, 3).map((source, index) => (
                      <div key={index} className={styles.sourceItem}>
                        <a
                          href={source}
                          target="_blank"
                          rel="noopener noreferrer"
                          className={styles.sourceLink}
                        >
                          {new URL(source).hostname}
                        </a>
                      </div>
                    ))}
                    {message.sources.length > 3 && (
                      <div className={styles.moreSources}>
                        ...and {message.sources.length - 3} more
                      </div>
                    )}
                  </div>
                )}
              </div>
            ))
          )}
          {isLoading && (
            <div className={clsx(styles.message, styles.agentMessage)}>
              <div className={styles.loadingIndicator}>
                <div className={styles.loadingDot}></div>
                <div className={styles.loadingDot}></div>
                <div className={styles.loadingDot}></div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {selectedText && (
          <div className={styles.selectedTextIndicator}>
            Selected text context included
          </div>
        )}

        <div className={styles.chatInputArea}>
          <div className={styles.inputContainer}>
            <textarea
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              className={styles.messageInput}
              rows={1}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={clsx(
                styles.sendBtn,
                !inputValue.trim() || isLoading ? styles.sendBtnDisabled : ''
              )}
            >
              <i className="fas fa-paper-plane"></i>
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default ChatWidget;