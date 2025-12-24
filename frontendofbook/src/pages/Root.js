import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

// Root component that wraps the entire application
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}