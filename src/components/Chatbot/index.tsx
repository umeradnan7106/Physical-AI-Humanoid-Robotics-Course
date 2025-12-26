import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    chapter: string;
    url: string;
    score: number;
  }>;
}

interface SelectionPopup {
  show: boolean;
  x: number;
  y: number;
  text: string;
}

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`);
  const [selectionPopup, setSelectionPopup] = useState<SelectionPopup>({ show: false, x: 0, y: 0, text: '' });
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const API_URL = process.env.NODE_ENV === 'production'
    ? 'https://physical-ai-humanoid-robotics-chatbot-backend-production.up.railway.app'
    : 'http://localhost:8000';

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Text selection handler
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 3) {
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectionPopup({
            show: true,
            x: rect.left + rect.width / 2,
            y: rect.top - 10,
            text: selectedText,
          });
        }
      } else {
        setSelectionPopup({ show: false, x: 0, y: 0, text: '' });
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const handleAskAI = () => {
    if (!selectionPopup.text) return;

    setIsOpen(true);
    setInput(`Explain this: "${selectionPopup.text}"`);
    setSelectionPopup({ show: false, x: 0, y: 0, text: '' });

    // Auto-send after a brief delay
    setTimeout(() => {
      sendMessageWithText(`Explain this: "${selectionPopup.text}"`);
    }, 300);
  };

  const formatMessage = (content: string) => {
    // Split by newlines and process
    const lines = content.split('\n');
    const formatted: JSX.Element[] = [];
    let currentList: string[] = [];
    let listType: 'bullet' | 'number' | null = null;

    const flushList = () => {
      if (currentList.length > 0) {
        formatted.push(
          listType === 'bullet' ? (
            <ul key={formatted.length}>
              {currentList.map((item, idx) => (
                <li key={idx} dangerouslySetInnerHTML={{ __html: item }} />
              ))}
            </ul>
          ) : (
            <ol key={formatted.length}>
              {currentList.map((item, idx) => (
                <li key={idx} dangerouslySetInnerHTML={{ __html: item }} />
              ))}
            </ol>
          )
        );
        currentList = [];
        listType = null;
      }
    };

    lines.forEach((line, index) => {
      const trimmed = line.trim();

      // Bullet points
      if (trimmed.startsWith('*') || trimmed.startsWith('-')) {
        if (listType !== 'bullet') {
          flushList();
          listType = 'bullet';
        }
        currentList.push(trimmed.substring(1).trim());
      }
      // Numbered lists
      else if (/^\d+\./.test(trimmed)) {
        if (listType !== 'number') {
          flushList();
          listType = 'number';
        }
        currentList.push(trimmed.replace(/^\d+\.\s*/, ''));
      }
      // Headings
      else if (trimmed.startsWith('#')) {
        flushList();
        const level = trimmed.match(/^#+/)?.[0].length || 1;
        const text = trimmed.replace(/^#+\s*/, '');
        const Tag = `h${Math.min(level + 2, 6)}` as keyof JSX.IntrinsicElements;
        formatted.push(<Tag key={index}>{text}</Tag>);
      }
      // Bold text
      else if (trimmed.includes('**')) {
        flushList();
        const parts = trimmed.split('**');
        formatted.push(
          <p key={index}>
            {parts.map((part, i) => (i % 2 === 0 ? part : <strong key={i}>{part}</strong>))}
          </p>
        );
      }
      // Regular paragraph
      else if (trimmed) {
        flushList();
        formatted.push(<p key={index}>{trimmed}</p>);
      }
    });

    flushList();
    return formatted.length > 0 ? formatted : <p>{content}</p>;
  };

  const sendMessageWithText = async (messageText: string) => {
    if (!messageText.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: messageText,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: messageText,
          session_id: sessionId,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        sources: data.sources,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: '😔 Oops! I ran into a problem. Make sure the backend server is running on http://localhost:8000 and try again!',
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const sendMessage = async () => {
    await sendMessageWithText(input);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Text Selection Popup */}
      {selectionPopup.show && (
        <div
          className={styles.askAiPopup}
          style={{
            left: `${selectionPopup.x}px`,
            top: `${selectionPopup.y}px`,
          }}
          onClick={handleAskAI}
        >
          ✨ Ask AI
        </div>
      )}

      {/* Chatbot Toggle Button */}
      <button
        className={styles.chatbotToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <h3>🤖 Your AI Study Buddy</h3>
            <p>I'm here to help you learn Physical AI & Robotics!</p>
          </div>

          {/* Messages */}
          <div className={styles.chatbotMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>👋 Hey there! I'm your friendly AI assistant!</p>
                <p>I can help you understand anything from this book. Try asking:</p>
                <ul>
                  <li onClick={() => sendMessageWithText("What is ROS 2?")}>
                    "What is ROS 2?"
                  </li>
                  <li onClick={() => sendMessageWithText("How do I create a URDF file?")}>
                    "How do I create a URDF file?"
                  </li>
                  <li onClick={() => sendMessageWithText("Explain Gazebo simulation")}>
                    "Explain Gazebo simulation"
                  </li>
                </ul>
                <div className={styles.tip}>
                  💡 <strong>Pro tip:</strong> Select any text on the page and click "Ask AI" to learn more!
                </div>
              </div>
            ) : (
              messages.map((message, index) => (
                <div
                  key={index}
                  className={`${styles.message} ${
                    message.role === 'user' ? styles.userMessage : styles.assistantMessage
                  }`}
                >
                  <div className={styles.messageContent}>
                    {message.role === 'assistant' ? formatMessage(message.content) : message.content}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className={styles.sources}>
                      <strong>📚 Sources:</strong>
                      {message.sources.map((source, idx) => (
                        <div key={idx} className={styles.source}>
                          📖 {source.chapter}
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typing}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                  <p className={styles.thinkingText}>Thinking...</p>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.chatbotInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask me anything about the book..."
              rows={1}
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !input.trim()}>
              {isLoading ? '...' : '➤'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;
