/**
 * @file socket_line_editor.cpp
 * @brief Socket-based line editor implementation
 *
 * ソケットベース行エディタの実装
 */

#include "socket_line_editor.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "lwip/sockets.h"

namespace stampfly {

// =============================================================================
// SocketCompletions
// =============================================================================

void SocketCompletions::add(const char* str)
{
    if (len >= MAX_COMPLETIONS || str == nullptr) {
        return;
    }
    cvec[len] = strdup(str);
    if (cvec[len] != nullptr) {
        len++;
    }
}

void SocketCompletions::clear()
{
    for (size_t i = 0; i < len; i++) {
        free(cvec[i]);
        cvec[i] = nullptr;
    }
    len = 0;
}

// =============================================================================
// Constructor / Destructor
// =============================================================================

// Telnet protocol constants
// Telnet プロトコル定数
static constexpr uint8_t IAC  = 255;  // Interpret As Command
static constexpr uint8_t WILL = 251;  // Will do option
static constexpr uint8_t WONT = 252;  // Won't do option
static constexpr uint8_t DO   = 253;  // Do option
static constexpr uint8_t DONT = 254;  // Don't do option
static constexpr uint8_t SB   = 250;  // Subnegotiation Begin
static constexpr uint8_t SE   = 240;  // Subnegotiation End

// Telnet options
static constexpr uint8_t TELOPT_ECHO     = 1;   // Echo
static constexpr uint8_t TELOPT_SGA      = 3;   // Suppress Go Ahead
static constexpr uint8_t TELOPT_LINEMODE = 34;  // Line Mode

SocketLineEditor::SocketLineEditor(int fd)
    : fd_(fd)
{
    memset(buffer_, 0, sizeof(buffer_));
    memset(history_, 0, sizeof(history_));
    memset(esc_buf_, 0, sizeof(esc_buf_));

    // Send telnet negotiation to enable character mode
    // キャラクタモードを有効にするためTelnetネゴシエーションを送信
    // Server WILL ECHO (we handle echo)
    // Server WILL SGA (suppress go-ahead)
    // Client should DO SGA
    uint8_t negotiate[] = {
        IAC, WILL, TELOPT_ECHO,      // Server will echo
        IAC, WILL, TELOPT_SGA,       // Server will suppress go-ahead
        IAC, DO,   TELOPT_SGA,       // Client should suppress go-ahead
        IAC, DONT, TELOPT_LINEMODE,  // Client should not use linemode
    };
    send(fd_, negotiate, sizeof(negotiate), 0);
}

SocketLineEditor::~SocketLineEditor()
{
    clearHistory();
}

void SocketLineEditor::clearHistory()
{
    for (int i = 0; i < history_len_; i++) {
        free(history_[i]);
        history_[i] = nullptr;
    }
    history_len_ = 0;
}

// =============================================================================
// Public Methods
// =============================================================================

char* SocketLineEditor::getLine(const char* prompt)
{
    // Reset line state
    // 行状態をリセット
    memset(buffer_, 0, sizeof(buffer_));
    pos_ = 0;
    len_ = 0;
    history_index_ = -1;
    esc_state_ = EscState::NONE;
    esc_pos_ = 0;

    // Send prompt
    // プロンプトを送信
    if (prompt != nullptr) {
        writeStr(prompt);
    }

    // Read characters
    // 文字を読み取る
    while (true) {
        char c;
        int ret = recv(fd_, &c, 1, 0);

        if (ret <= 0) {
            // Error or disconnect
            // エラーまたは切断
            return nullptr;
        }

        // Process character
        // 文字を処理
        if (processChar(c, prompt)) {
            // Line complete
            // 行完了
            buffer_[len_] = '\0';
            writeStr("\r\n");
            return strdup(buffer_);
        }
    }
}

void SocketLineEditor::freeLine(char* line)
{
    free(line);
}

void SocketLineEditor::addHistory(const char* line)
{
    if (line == nullptr || line[0] == '\0') {
        return;
    }

    // Don't add duplicate of most recent entry
    // 直近のエントリと重複する場合は追加しない
    if (history_len_ > 0 && strcmp(history_[history_len_ - 1], line) == 0) {
        return;
    }

    // If at max, remove oldest entry
    // 最大に達していたら最古のエントリを削除
    if (history_len_ >= history_max_) {
        free(history_[0]);
        memmove(history_, history_ + 1, (history_max_ - 1) * sizeof(char*));
        history_len_--;
    }

    // Add new entry
    // 新しいエントリを追加
    history_[history_len_] = strdup(line);
    if (history_[history_len_] != nullptr) {
        history_len_++;
    }
}

void SocketLineEditor::setHistoryMaxLen(int len)
{
    if (len < 1) len = 1;
    if (len > MAX_HISTORY) len = MAX_HISTORY;
    history_max_ = len;

    // Trim existing history if needed
    // 必要に応じて既存の履歴を削減
    while (history_len_ > history_max_) {
        free(history_[0]);
        memmove(history_, history_ + 1, (history_len_ - 1) * sizeof(char*));
        history_len_--;
    }
}

void SocketLineEditor::setCompletionCallback(SocketCompletionCallback callback)
{
    completion_callback_ = callback;
}

// =============================================================================
// Character Processing
// =============================================================================

bool SocketLineEditor::processChar(char c, const char* prompt)
{
    uint8_t uc = static_cast<uint8_t>(c);

    // Handle telnet IAC sequences
    // Telnet IAC シーケンスを処理
    if (esc_state_ == EscState::IAC) {
        if (uc == IAC) {
            // IAC IAC = escaped 255 byte, treat as data
            // IAC IAC = エスケープされた255バイト、データとして扱う
            esc_state_ = EscState::NONE;
            // Fall through to insert as character (rare case)
        } else if (uc == WILL || uc == WONT || uc == DO || uc == DONT) {
            // Two-byte command, wait for option byte
            // 2バイトコマンド、オプションバイトを待つ
            esc_state_ = EscState::IAC_CMD;
            esc_buf_[0] = c;
            return false;
        } else if (uc == SB) {
            // Subnegotiation, skip until IAC SE
            // サブネゴシエーション、IAC SEまでスキップ
            esc_state_ = EscState::IAC_SB;
            return false;
        } else {
            // Unknown IAC command, ignore
            // 不明なIACコマンド、無視
            esc_state_ = EscState::NONE;
            return false;
        }
    }
    if (esc_state_ == EscState::IAC_CMD) {
        // Got option byte, ignore and reset
        // オプションバイト取得、無視してリセット
        esc_state_ = EscState::NONE;
        return false;
    }
    if (esc_state_ == EscState::IAC_SB) {
        // In subnegotiation, look for IAC
        // サブネゴシエーション中、IACを探す
        if (uc == IAC) {
            esc_state_ = EscState::IAC_SB_IAC;
        }
        return false;
    }
    if (esc_state_ == EscState::IAC_SB_IAC) {
        // Got IAC in subnegotiation
        if (uc == SE) {
            // End of subnegotiation
            esc_state_ = EscState::NONE;
        } else {
            // Not SE, continue in SB mode
            esc_state_ = EscState::IAC_SB;
        }
        return false;
    }

    // Check for IAC start
    // IAC開始をチェック
    if (uc == IAC) {
        esc_state_ = EscState::IAC;
        return false;
    }

    // Handle escape sequences
    // エスケープシーケンスを処理
    if (esc_state_ == EscState::ESC) {
        processEscape(c, prompt);
        return false;
    }
    if (esc_state_ == EscState::CSI) {
        processCSI(c, prompt);
        return false;
    }

    // Regular character processing
    // 通常の文字処理
    switch (c) {
        case '\r':
        case '\n':
            // Enter - line complete
            // Enter - 行完了
            return true;

        case 0x1B:  // ESC
            esc_state_ = EscState::ESC;
            esc_pos_ = 0;
            return false;

        case 0x7F:  // DEL
        case '\b':  // Backspace (Ctrl+H)
            handleBackspace(prompt);
            return false;

        case 0x01:  // Ctrl+A - Home
            handleHome(prompt);
            return false;

        case 0x02:  // Ctrl+B - Left
            handleArrowLeft(prompt);
            return false;

        case 0x03:  // Ctrl+C - Cancel
            writeStr("^C\r\n");
            if (prompt) writeStr(prompt);
            len_ = 0;
            pos_ = 0;
            buffer_[0] = '\0';
            return false;

        case 0x04:  // Ctrl+D - EOF (if empty) or delete
            if (len_ == 0) {
                return true;  // Treat as disconnect
            }
            handleDelete(prompt);
            return false;

        case 0x05:  // Ctrl+E - End
            handleEnd(prompt);
            return false;

        case 0x06:  // Ctrl+F - Right
            handleArrowRight(prompt);
            return false;

        case '\t':  // Tab - completion
            handleTab(prompt);
            return false;

        case 0x0B:  // Ctrl+K - Kill to end of line
            // Delete from cursor to end
            // カーソルから末尾まで削除
            if (pos_ < len_) {
                len_ = pos_;
                buffer_[len_] = '\0';
                refreshLine(prompt);
            }
            return false;

        case 0x0C:  // Ctrl+L - Clear screen
            writeStr("\x1B[2J\x1B[H");  // Clear and home
            refreshLine(prompt);
            return false;

        case 0x0E:  // Ctrl+N - Down (next history)
            handleArrowDown(prompt);
            return false;

        case 0x10:  // Ctrl+P - Up (previous history)
            handleArrowUp(prompt);
            return false;

        case 0x15:  // Ctrl+U - Kill line
            len_ = 0;
            pos_ = 0;
            buffer_[0] = '\0';
            refreshLine(prompt);
            return false;

        case 0x17:  // Ctrl+W - Kill word
            // Delete previous word
            // 前の単語を削除
            while (pos_ > 0 && buffer_[pos_ - 1] == ' ') {
                deleteCharAt(pos_ - 1, prompt);
            }
            while (pos_ > 0 && buffer_[pos_ - 1] != ' ') {
                deleteCharAt(pos_ - 1, prompt);
            }
            return false;

        default:
            // Printable character
            // 印字可能文字
            if (c >= 0x20 && c < 0x7F) {
                insertChar(c, prompt);
            }
            return false;
    }
}

void SocketLineEditor::processEscape(char c, const char* prompt)
{
    if (c == '[') {
        esc_state_ = EscState::CSI;
        return;
    }
    if (c == 'O') {
        // SS3 sequence (some terminals use this)
        // SS3シーケンス（一部のターミナルで使用）
        esc_state_ = EscState::CSI;
        return;
    }
    // Unknown escape, reset
    // 不明なエスケープ、リセット
    esc_state_ = EscState::NONE;
}

void SocketLineEditor::processCSI(char c, const char* prompt)
{
    // CSI sequences: ESC [ <params> <final_byte>
    // Final byte is in range 0x40-0x7E
    // CSIシーケンス: ESC [ <パラメータ> <最終バイト>

    if (c >= '0' && c <= '9') {
        // Parameter byte
        // パラメータバイト
        if (esc_pos_ < (int)sizeof(esc_buf_) - 1) {
            esc_buf_[esc_pos_++] = c;
        }
        return;
    }
    if (c == ';') {
        // Parameter separator
        // パラメータセパレータ
        if (esc_pos_ < (int)sizeof(esc_buf_) - 1) {
            esc_buf_[esc_pos_++] = c;
        }
        return;
    }

    // Final byte
    // 最終バイト
    esc_state_ = EscState::NONE;
    esc_buf_[esc_pos_] = '\0';

    switch (c) {
        case 'A':  // Up arrow
            handleArrowUp(prompt);
            break;
        case 'B':  // Down arrow
            handleArrowDown(prompt);
            break;
        case 'C':  // Right arrow
            handleArrowRight(prompt);
            break;
        case 'D':  // Left arrow
            handleArrowLeft(prompt);
            break;
        case 'H':  // Home
            handleHome(prompt);
            break;
        case 'F':  // End
            handleEnd(prompt);
            break;
        case '~':
            // Extended codes: 1~ Home, 3~ Delete, 4~ End, etc.
            // 拡張コード: 1~ Home, 3~ Delete, 4~ End など
            if (esc_pos_ > 0) {
                int code = atoi(esc_buf_);
                switch (code) {
                    case 1:  // Home
                        handleHome(prompt);
                        break;
                    case 3:  // Delete
                        handleDelete(prompt);
                        break;
                    case 4:  // End
                        handleEnd(prompt);
                        break;
                }
            }
            break;
    }

    esc_pos_ = 0;
}

// =============================================================================
// Arrow Key Handlers
// =============================================================================

void SocketLineEditor::handleArrowUp(const char* prompt)
{
    if (history_len_ == 0) {
        return;
    }

    // First up press: start from most recent
    // 最初の上押下: 最新から開始
    if (history_index_ < 0) {
        history_index_ = history_len_ - 1;
    } else if (history_index_ > 0) {
        history_index_--;
    } else {
        return;  // Already at oldest
    }

    // Copy history entry to buffer
    // 履歴エントリをバッファにコピー
    strncpy(buffer_, history_[history_index_], BUFFER_SIZE - 1);
    buffer_[BUFFER_SIZE - 1] = '\0';
    len_ = strlen(buffer_);
    pos_ = len_;

    refreshLine(prompt);
}

void SocketLineEditor::handleArrowDown(const char* prompt)
{
    if (history_index_ < 0) {
        return;
    }

    if (history_index_ < history_len_ - 1) {
        history_index_++;
        strncpy(buffer_, history_[history_index_], BUFFER_SIZE - 1);
        buffer_[BUFFER_SIZE - 1] = '\0';
    } else {
        // Past most recent - clear line
        // 最新を過ぎた - 行をクリア
        history_index_ = -1;
        buffer_[0] = '\0';
    }
    len_ = strlen(buffer_);
    pos_ = len_;

    refreshLine(prompt);
}

void SocketLineEditor::handleArrowLeft(const char* prompt)
{
    (void)prompt;
    if (pos_ > 0) {
        pos_--;
        writeStr("\x1B[D");  // Move cursor left
    }
}

void SocketLineEditor::handleArrowRight(const char* prompt)
{
    (void)prompt;
    if (pos_ < len_) {
        pos_++;
        writeStr("\x1B[C");  // Move cursor right
    }
}

void SocketLineEditor::handleHome(const char* prompt)
{
    (void)prompt;
    if (pos_ > 0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "\x1B[%dD", (int)pos_);
        writeStr(buf);
        pos_ = 0;
    }
}

void SocketLineEditor::handleEnd(const char* prompt)
{
    (void)prompt;
    if (pos_ < len_) {
        char buf[16];
        snprintf(buf, sizeof(buf), "\x1B[%dC", (int)(len_ - pos_));
        writeStr(buf);
        pos_ = len_;
    }
}

// =============================================================================
// Edit Operations
// =============================================================================

void SocketLineEditor::handleBackspace(const char* prompt)
{
    if (pos_ > 0) {
        deleteCharAt(pos_ - 1, prompt);
    }
}

void SocketLineEditor::handleDelete(const char* prompt)
{
    if (pos_ < len_) {
        // Delete character at cursor
        // カーソル位置の文字を削除
        memmove(buffer_ + pos_, buffer_ + pos_ + 1, len_ - pos_);
        len_--;
        refreshLine(prompt);
    }
}

void SocketLineEditor::handleTab(const char* prompt)
{
    if (completion_callback_ == nullptr) {
        return;
    }

    // Get completions
    // 補完候補を取得
    SocketCompletions completions;
    buffer_[len_] = '\0';
    completion_callback_(buffer_, &completions);

    if (completions.len == 0) {
        // No completions - beep
        // 補完なし - ビープ
        writeStr("\x07");
    } else if (completions.len == 1) {
        // Single completion - insert it
        // 単一の補完 - 挿入
        strncpy(buffer_, completions.cvec[0], BUFFER_SIZE - 1);
        buffer_[BUFFER_SIZE - 1] = '\0';
        len_ = strlen(buffer_);
        pos_ = len_;
        refreshLine(prompt);
    } else {
        // Multiple completions - show them
        // 複数の補完 - 表示
        writeStr("\r\n");
        for (size_t i = 0; i < completions.len; i++) {
            writeStr(completions.cvec[i]);
            writeStr("  ");
        }
        writeStr("\r\n");
        refreshLine(prompt);
    }

    completions.clear();
}

void SocketLineEditor::insertChar(char c, const char* prompt)
{
    if (len_ >= BUFFER_SIZE - 1) {
        return;
    }

    if (pos_ == len_) {
        // Insert at end - simple append
        // 末尾に挿入 - 単純な追加
        buffer_[pos_] = c;
        pos_++;
        len_++;
        buffer_[len_] = '\0';
        writeChar(c);
    } else {
        // Insert in middle - need to shift
        // 中間に挿入 - シフトが必要
        memmove(buffer_ + pos_ + 1, buffer_ + pos_, len_ - pos_);
        buffer_[pos_] = c;
        pos_++;
        len_++;
        buffer_[len_] = '\0';
        refreshLine(prompt);
    }
}

void SocketLineEditor::deleteCharAt(size_t del_pos, const char* prompt)
{
    if (del_pos >= len_) {
        return;
    }

    memmove(buffer_ + del_pos, buffer_ + del_pos + 1, len_ - del_pos);
    len_--;
    if (pos_ > del_pos) {
        pos_--;
    }
    refreshLine(prompt);
}

// =============================================================================
// Output Helpers
// =============================================================================

void SocketLineEditor::writeStr(const char* str)
{
    if (str == nullptr || fd_ < 0) return;
    size_t len = strlen(str);
    if (len > 0) {
        send(fd_, str, len, 0);
    }
}

void SocketLineEditor::writeChar(char c)
{
    if (fd_ >= 0) {
        send(fd_, &c, 1, 0);
    }
}

void SocketLineEditor::refreshLine(const char* prompt)
{
    // Build the entire output string first, then send at once
    // 出力文字列を先に構築し、一度に送信（チラつき軽減）
    char output[512];
    int out_pos = 0;

    // Save cursor, move to beginning, clear line
    // カーソル保存、行先頭に移動、行クリア
    // Use \x1B[?25l to hide cursor, \x1B[?25h to show
    const char* prefix = "\x1B[?25l\r\x1B[K";  // Hide cursor + CR + clear line
    size_t prefix_len = strlen(prefix);
    if (out_pos + prefix_len < sizeof(output)) {
        memcpy(output + out_pos, prefix, prefix_len);
        out_pos += prefix_len;
    }

    // Add prompt
    // プロンプトを追加
    if (prompt) {
        size_t plen = strlen(prompt);
        if (out_pos + plen < sizeof(output)) {
            memcpy(output + out_pos, prompt, plen);
            out_pos += plen;
        }
    }

    // Add buffer content
    // バッファ内容を追加
    if (len_ > 0 && out_pos + len_ < sizeof(output)) {
        memcpy(output + out_pos, buffer_, len_);
        out_pos += len_;
    }

    // Move cursor to correct position if not at end
    // カーソルが末尾でない場合、正しい位置に移動
    if (pos_ < len_) {
        char move_buf[16];
        int move_len = snprintf(move_buf, sizeof(move_buf), "\x1B[%dD", (int)(len_ - pos_));
        if (out_pos + move_len < (int)sizeof(output)) {
            memcpy(output + out_pos, move_buf, move_len);
            out_pos += move_len;
        }
    }

    // Show cursor again
    // カーソルを再表示
    const char* suffix = "\x1B[?25h";
    size_t suffix_len = strlen(suffix);
    if (out_pos + suffix_len < sizeof(output)) {
        memcpy(output + out_pos, suffix, suffix_len);
        out_pos += suffix_len;
    }

    // Send all at once
    // 一度に送信
    if (out_pos > 0) {
        send(fd_, output, out_pos, 0);
    }
}

}  // namespace stampfly
