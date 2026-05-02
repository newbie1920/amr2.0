/**
 * AMR 2.0 — React Error Boundary
 * Bắt lỗi runtime trong component tree, hiển thị fallback UI
 * thay vì trang trắng. Hỗ trợ retry và report lỗi.
 */

import { Component } from 'react';

class ErrorBoundary extends Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null, errorInfo: null };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }

  componentDidCatch(error, errorInfo) {
    this.setState({ errorInfo });
    console.error('[ErrorBoundary] Caught error:', error, errorInfo);
  }

  handleRetry = () => {
    this.setState({ hasError: false, error: null, errorInfo: null });
  };

  render() {
    if (this.state.hasError) {
      return (
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          minHeight: '100vh',
          background: '#0d1117',
          color: '#e6edf3',
          fontFamily: "'Inter', -apple-system, sans-serif",
          padding: '2rem',
        }}>
          <div style={{
            maxWidth: '560px',
            background: '#161b22',
            border: '1px solid #30363d',
            borderRadius: '12px',
            padding: '2rem',
            textAlign: 'center',
            boxShadow: '0 8px 32px rgba(0,0,0,0.4)',
          }}>
            <div style={{ fontSize: '48px', marginBottom: '16px' }}>⚠️</div>
            <h2 style={{ margin: '0 0 8px', fontSize: '20px', color: '#f85149' }}>
              Đã xảy ra lỗi ứng dụng
            </h2>
            <p style={{ color: '#8b949e', fontSize: '14px', margin: '0 0 16px' }}>
              NavTDTU gặp lỗi không mong muốn. Bạn có thể thử tải lại.
            </p>
            
            <details style={{
              textAlign: 'left',
              background: '#0d1117',
              border: '1px solid #30363d',
              borderRadius: '8px',
              padding: '12px',
              marginBottom: '16px',
              fontSize: '12px',
              color: '#8b949e',
            }}>
              <summary style={{ cursor: 'pointer', color: '#58a6ff', marginBottom: '8px' }}>
                Chi tiết lỗi
              </summary>
              <pre style={{
                whiteSpace: 'pre-wrap',
                wordBreak: 'break-all',
                margin: 0,
                maxHeight: '200px',
                overflow: 'auto',
              }}>
                {this.state.error?.toString()}
                {'\n\n'}
                {this.state.errorInfo?.componentStack}
              </pre>
            </details>

            <div style={{ display: 'flex', gap: '12px', justifyContent: 'center' }}>
              <button
                onClick={this.handleRetry}
                style={{
                  padding: '10px 24px',
                  background: '#238636',
                  color: '#fff',
                  border: 'none',
                  borderRadius: '8px',
                  fontSize: '14px',
                  fontWeight: 600,
                  cursor: 'pointer',
                  transition: 'background 0.2s',
                }}
                onMouseOver={(e) => e.target.style.background = '#2ea043'}
                onMouseOut={(e) => e.target.style.background = '#238636'}
              >
                🔄 Thử lại
              </button>
              <button
                onClick={() => window.location.reload()}
                style={{
                  padding: '10px 24px',
                  background: '#21262d',
                  color: '#c9d1d9',
                  border: '1px solid #30363d',
                  borderRadius: '8px',
                  fontSize: '14px',
                  fontWeight: 600,
                  cursor: 'pointer',
                  transition: 'background 0.2s',
                }}
                onMouseOver={(e) => e.target.style.background = '#30363d'}
                onMouseOut={(e) => e.target.style.background = '#21262d'}
              >
                🔃 Tải lại trang
              </button>
            </div>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

export default ErrorBoundary;
